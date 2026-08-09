#include "ntrip_client.h"
#include "device_config.h"
#include "gnss.h"
#include "modem.h"
#include "geophone.h"
#include <string.h>

static bool connected = false;
static uint32_t bytesForwarded = 0;

static String base64Encode(const String &input) {
  static const char table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  String out;
  int len = input.length();

  for (int i = 0; i < len; i += 3) {
    uint32_t octet_a = (uint8_t)input[i];
    uint32_t octet_b = i + 1 < len ? (uint8_t)input[i + 1] : 0;
    uint32_t octet_c = i + 2 < len ? (uint8_t)input[i + 2] : 0;
    uint32_t triple = (octet_a << 16) + (octet_b << 8) + octet_c;

    out += table[(triple >> 18) & 0x3F];
    out += table[(triple >> 12) & 0x3F];
    out += (i + 1 < len) ? table[(triple >> 6) & 0x3F] : '=';
    out += (i + 2 < len) ? table[triple & 0x3F] : '=';
  }
  return out;
}

// Reads one line (up to and including '\n') from the modem socket, blocking
// briefly. Used only during the header-parsing part of the NTRIP handshake -
// the streaming RTCM body afterward goes through ntripPoll() instead.
static String socketReadLine(unsigned long timeoutMs) {
  String line;
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    if (modemSocketAvailable() > 0) {
      uint8_t b;
      modemSocketRead(&b, 1);
      line += (char)b;
      if (b == '\n') {
        return line;
      }
    } else {
      geophoneAwareDelay(10);
    }
  }
  return line;
}

// Single connection attempt. *wasUnauthorized is set so the caller can
// pick a shorter retry delay for a 401 - observed on this caster to often
// be a transient rejection that a quick retry clears, not a genuine
// credentials problem.
static bool ntripConnectOnce(bool *wasUnauthorized) {
  *wasUnauthorized = false;
  Serial.printf("Connecting to NTRIP caster %s:%d ...\n", deviceConfig.ntrip_host, deviceConfig.ntrip_port);

  if (!modemSocketOpen(deviceConfig.ntrip_host, deviceConfig.ntrip_port)) {
    Serial.println(F("NTRIP TCP connect failed."));
    return false;
  }

  String auth = base64Encode(String(deviceConfig.ntrip_user) + ":" + String(deviceConfig.ntrip_password));

  String request = String("GET /") + deviceConfig.ntrip_mountpoint + " HTTP/1.0\r\n" +
                    "Host: " + deviceConfig.ntrip_host + "\r\n" +
                    "User-Agent: NTRIP WalterClient/1.0\r\n" +
                    "Authorization: Basic " + auth + "\r\n" +
                    "\r\n";
  modemSocketSend((const uint8_t *)request.c_str(), request.length());

  String statusLine = socketReadLine(5000);
  if (statusLine.length() == 0) {
    Serial.println(F("NTRIP caster did not respond."));
    modemSocketClose();
    return false;
  }

  Serial.print(F("Caster response: "));
  Serial.println(statusLine);

  if (statusLine.indexOf("200") < 0) {
    if (statusLine.indexOf("401") >= 0) {
      Serial.println(F("NTRIP caster returned 401 Unauthorized - retrying shortly (often transient)."));
      *wasUnauthorized = true;
    } else {
      Serial.println(F("NTRIP caster rejected request (check mountpoint/credentials)."));
    }
    modemSocketClose();
    return false;
  }

  // Drain the rest of the HTTP header block up to the blank line.
  while (true) {
    String headerLine = socketReadLine(2000);
    if (headerLine == "\r\n" || headerLine.length() == 0) {
      break;
    }
  }

  Serial.println(F("NTRIP stream connected, forwarding RTCM to F9P."));
  connected = true;
  return true;
}

bool ntripConnect() {
  // A 401 on this caster isn't a real credentials problem - confirmed by
  // the user restarting and having it work again shortly after - but 3
  // retries at a flat 5s (10s total) wasn't long enough: live hardware hit
  // 401 on all 3 attempts in a row. This looks like caster-side session
  // state (e.g. a previous connection not yet timed out server-side)
  // needing real wall-clock time to clear, not something a quick retry
  // fixes. Widened the budget and made the delay increase each attempt,
  // so a slow-to-clear session gets a real chance without permanently
  // blocking the fix cycle - worst case (5 attempts, all 401) is 5+15+30+45
  // = 95s of retrying, still leaving the GNSS fix-wait loop's own timeout
  // as the outer bound. A non-401 failure keeps a fixed, shorter delay -
  // no evidence it needs the same treatment.
  static const int MAX_ATTEMPTS = 5;
  static const unsigned long RETRY_DELAY_UNAUTHORIZED_MS[MAX_ATTEMPTS - 1] = {5000, 15000, 30000, 45000};
  static const unsigned long RETRY_DELAY_GENERAL_MS = 10000;

  for (int attempt = 1; attempt <= MAX_ATTEMPTS; attempt++) {
    bool wasUnauthorized = false;
    if (ntripConnectOnce(&wasUnauthorized)) {
      return true;
    }

    if (attempt == MAX_ATTEMPTS) {
      break;
    }

    unsigned long delayMs = wasUnauthorized ? RETRY_DELAY_UNAUTHORIZED_MS[attempt - 1] : RETRY_DELAY_GENERAL_MS;
    Serial.printf("NTRIP connect attempt %d/%d failed - retrying in %lus...\n", attempt, MAX_ATTEMPTS,
                  delayMs / 1000);
    geophoneAwareDelay(delayMs);
  }

  Serial.println(F("NTRIP connect: giving up for this fix cycle."));
  return false;
}

bool ntripIsConnected() {
  return connected;
}

void ntripPoll() {
  uint8_t buf[256];
  size_t n;
  while ((n = modemSocketRead(buf, sizeof(buf))) > 0) {
    for (size_t i = 0; i < n; i++) {
      gnssWriteRTCM(buf[i]);
    }
    bytesForwarded += n;
  }
}

bool ntripSendGGA(const char *ggaLine) {
  if (!connected || ggaLine[0] == '\0') {
    return false;
  }
  // A send submitted while a receive is in flight was found to reliably
  // collide on WalterModem's command channel and fail with "+CME ERROR:
  // 4" - live hardware showed 100% of GGA sends failing this way. Defer
  // rather than send into a near-certain failure; the caller retries soon
  // without waiting a full resend interval.
  if (modemReceiveBusy()) {
    return false;
  }
  // Checking "not busy right now" alone still only got ~29% of sends
  // through on live hardware, because RTCM now drains almost
  // continuously - a fresh receive can get kicked off in the gap between
  // this check and the actual send. Claim the channel for a brief window
  // so no *new* receive starts during it.
  modemSuppressReceiveFor(300);
  String line = String(ggaLine) + "\r\n";
  modemSocketSend((const uint8_t *)line.c_str(), line.length());
  return true;
}

uint32_t ntripBytesForwarded() {
  return bytesForwarded;
}

void ntripDisconnect() {
  modemSocketClose();
  connected = false;
}
