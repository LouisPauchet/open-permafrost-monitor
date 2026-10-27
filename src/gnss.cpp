#include "gnss.h"
#include "pins.h"
#include "sd_storage.h"
#include <TinyGPSPlus.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

// Mirror every raw byte from the F9P to the USB serial monitor - useful for
// debugging (e.g. confirming GST/NMEA output timing around EXTINT wake), but
// costly: live hardware crashed (task watchdog firing, main task spending
// too long blocked in Serial writes) once RTCM started flowing heavily at
// the same time as this byte-by-byte NMEA mirror and WalterModem's own AT
// debug tracing - all competing for the same USB-CDC link. Flip back to
// true only for short, targeted debugging sessions, not left on during
// real operation.
static const bool DEBUG_PRINT_RAW_GNSS = false;

// UART0, not UART2: the Arduino core's global `Serial2` object (used by
// modem.cpp for the Sequans modem) is ALSO hardware UART peripheral 2
// (HardwareSerial Serial2(2), core/HardwareSerial.cpp) - two HardwareSerial
// C++ instances both bound to uart_nr=2 share the same underlying IDF UART
// driver, ring buffer, and GPIO matrix routing, not just the same "logical"
// port. modemInit() (Serial2.begin(), pins 14/48) runs after initGNSS()
// (this begin(), pins 16/17) in setup() - IDF's uart begin() detaches
// whichever pins were previously attached and re-routes the peripheral to
// the new ones, so from that point on this object would have been reading/
// writing the modem's physical pins instead of the F9P's. That silently
// explains a lot of this project's history: the F9P's own UBX replies
// (RXM-RTCM, ACK/NAK) never being seen once the modem initializes, RTCM
// meant for the F9P instead hitting the modem's RX line and corrupting its
// AT command channel, and GNSS fix acquisition getting starved. UART0 is
// genuinely free to use here - platformio.ini sets
// ARDUINO_USB_CDC_ON_BOOT=1, so the console runs over native USB, not
// physical UART0 (pins.h's own comment on FIELD_MODE_SELECT_PIN already
// notes UART0/IO44/IO43 as "exposed but unused").
static HardwareSerial GPSSerial(0);
static TinyGPSPlus gps;

// GST (GNSS Pseudorange error statistics) carries the receiver's own 1-sigma
// position error estimate in meters - a real accuracy figure, not just HDOP.
// It's off by default on the F9P, enabled below with a UBX-CFG-MSG command.
// Field numbers are 1-based, counted after the talker+sentence ID:
// $GNGST,time,rangeRms,stdMajor,stdMinor,orient,stdLat,stdLon,stdAlt*cs
static TinyGPSCustom gstStdLat(gps, "GNGST", 6);
static TinyGPSCustom gstStdLon(gps, "GNGST", 7);
static TinyGPSCustom gstStdAlt(gps, "GNGST", 8);

// GGA fix quality: 0=no fix, 1=autonomous, 2=DGPS, 4=RTK fixed, 5=RTK float.
// $GNGGA,time,lat,NS,lon,EW,quality,numSV,hdop,alt,M,geoidSep,M,diffAge,diffStation*cs
static TinyGPSCustom ggaFixQuality(gps, "GNGGA", 6);

// UBX-CFG-MSG: enable NMEA GST (class 0xF0, id 0x07) on UART1 only, rate 1.
// This setting lives in RAM only, so it's resent every boot - nothing to undo.
static const uint8_t UBX_ENABLE_GST_UART1[] = {
  0xB5, 0x62,             // sync chars
  0x06, 0x01,             // class/id: CFG-MSG
  0x08, 0x00,             // payload length = 8
  0xF0, 0x07,             // msgClass=NMEA, msgID=GST
  0x00, 0x01, 0x00, 0x00, 0x00, 0x00, // rates: DDC, UART1, UART2, USB, SPI, reserved
  0x07, 0x59              // checksum (CK_A, CK_B)
};

// UBX-CFG-MSG: enable RXM-RAWX (class 0x02, id 0x15) and RXM-SFRBX
// (class 0x02, id 0x13) on UART1 - raw pseudorange/carrier-phase and
// broadcast navigation data, needed for PPK/PPP post-processing. Only sent
// when SD logging is actually available, so nothing changes on the F9P
// until the SD reader is wired up.
static const uint8_t UBX_ENABLE_RAWX_UART1[] = {
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
  0x02, 0x15, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
  0x27, 0x4B
};
static const uint8_t UBX_ENABLE_SFRBX_UART1[] = {
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
  0x02, 0x13, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
  0x25, 0x3D
};

// UBX-CFG-VALSET: set CFG-UART1INPROT-RTCM3X (key 0x10730004) = 1 (RAM
// layer only), i.e. explicitly enable RTCM3 as an *input* protocol on
// UART1. This is the likely real root cause behind RTK never engaging
// despite 90KB+/cycle of RTCM being forwarded and GGA reaching the caster:
// if UART1's input protocol mask doesn't include RTCM3, the F9P discards
// every RTCM byte before ever attempting to parse it - which would also
// explain why the RXM-RTCM diagnostic below never reports anything, not
// even a CRC failure, despite retrying its own enable on a timer. Setting
// this doesn't touch UBX/NMEA input (VALSET only touches the keys listed),
// so nothing else about the existing config changes.
static const uint8_t UBX_SET_UART1_INPROT_RTCM3X[] = {
  0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00,
  0x00, 0x01, 0x00, 0x00,             // version=0, layers=RAM, transaction=0, reserved0
  0x04, 0x00, 0x73, 0x10,             // keyID 0x10730004 (CFG-UART1INPROT-RTCM3X), little-endian
  0x01,                               // value = 1 (enabled)
  0x22, 0xBF                          // checksum (CK_A, CK_B)
};

// UBX-CFG-MSG: enable RXM-RTCM (class 0x02, id 0x32) on UART1 - diagnostic
// only, temporary while chasing why the fix stays autonomous despite a
// healthy volume of RTCM being forwarded. RXM-RTCM is the F9P's own report
// of what it did with each incoming RTCM message (type + CRC pass/fail),
// which is ground truth on whether corrections are actually usable versus
// just "bytes were sent" - see gnssParseRxmRtcmByte() below.
static const uint8_t UBX_ENABLE_RXM_RTCM_UART1[] = {
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
  0x02, 0x32, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
  0x44, 0x16
};

// UBX-RXM-PMREQ (class 0x02, id 0x41), 16-byte payload version, requesting
// indefinite backup mode (duration=0), force+backup flags (0x06 - enters
// backup even with USB attached, so bring-up over USB serial behaves the
// same as field/battery operation), waking on EXTINT0 only. Byte layout and
// wakeupSources bit values (EXTINT0=0x20, UARTRX=0x08) verified against
// SparkFun's u-blox GNSS Arduino library
// (src/SparkFun_u-blox_GNSS_Arduino_Library.cpp, powerOffWithInterrupt()),
// checksum computed by hand.
//
// UARTRX was originally also set as a wake source ("a bring-up safety net
// in case EXTINT wiring/polarity needs adjustment") - live hardware showed
// this defeating sleep entirely: the module never actually stayed in backup
// mode (gnssCharsProcessed() kept climbing throughout the "idle" period
// between cycles, confirmed on a station that's supposed to sleep the F9P
// ~80% of each measurement_interval_s). UART RX as a wake source means the
// module treats *any* electrical activity on the shared line - not just a
// valid command - as a wake trigger, including noise. EXTINT wake (see
// gnssWakeup()) has been the actual, working, per-cycle wake mechanism the
// whole time regardless; the "safety net" was never load-bearing and was
// actively breaking sleep. Removed.
static const uint8_t UBX_PMREQ_BACKUP_EXTINT[] = {
  0xB5, 0x62, 0x02, 0x41, 0x10, 0x00,
  0x00, 0x00, 0x00, 0x00,             // version, reserved
  0x00, 0x00, 0x00, 0x00,             // duration = 0 (indefinite)
  0x06, 0x00, 0x00, 0x00,             // flags: force | backup
  0x20, 0x00, 0x00, 0x00,             // wakeupSources: EXTINT0 only
  0x79, 0xCB                          // checksum (CK_A, CK_B)
};

// Raw GGA line last seen from the F9P itself, relayed to the NTRIP caster
// as-is (correct format/precision/checksum guaranteed since the module made it).
static char lastGGA[100] = "";
static bool haveGGA = false;
static char lineBuf[100];
static uint8_t lineLen = 0;

// gnssYear()/gnssHour()/etc. (and gnssEpochMillis(), built from them) return
// TinyGPS++'s last-received value - it doesn't expire on its own (isValid()
// is a sticky flag, confirmed against the library source, not an age check),
// so those keep reporting whatever the wall clock was at the *last* fix
// forever, including all through GNSS backup-mode sleep between cycles. A
// caller that stamps something with "the current time" using them directly
// (miniseed_writer.cpp did) gets the same frozen timestamp repeated for
// every sample recorded during that entire idle window - confirmed on live
// hardware: once GNSS sleep actually started working tonight (see
// gnssSleep()'s own fix), the geophone's continuous recording requirement
// broke as a direct, unintended side effect, because every idle-period
// MiniSEED record collided at the same instant instead of advancing.
// gnssEpochMillisNow()/gnssCivilNow() below extrapolate off millis() from
// the last genuinely fresh fix instead, so time keeps advancing correctly
// even while the F9P is asleep and not reporting at all.
// Written on loopTask (pollGNSS()), read on the geophone task (MiniSEED
// timing) - a 64-bit value plus its millis() pair can't be read atomically,
// so both sides go through anchorMux.
static portMUX_TYPE anchorMux = portMUX_INITIALIZER_UNLOCKED;
static uint64_t epochAnchorMs = 0;
static unsigned long epochAnchorAtMillis = 0;
static volatile bool haveEpochAnchor = false;
static uint16_t lastAnchoredYear = 0;
static uint8_t lastAnchoredMonth = 0, lastAnchoredDay = 0, lastAnchoredHour = 0,
               lastAnchoredMinute = 0, lastAnchoredSecond = 0;
static void maybeUpdateEpochAnchor(); // defined below, after gnssEpochMillis()
// Only sentences that were read promptly after arriving may anchor the
// clock: if loopTask was stuck in a blocking call (modem AT command), the
// backlog is parsed all at once and its time fields describe a moment up to
// the whole stall ago - anchoring "now" to them would step the clock back,
// which the MiniSEED timing would then have to re-sync to (and back).
static const unsigned long ANCHOR_MAX_POLL_GAP_MS = 100;
static unsigned long lastPollGnssMs = 0;

// Two capture files per GNSS burst (wake-to-sleep cycle, see
// gnssWakeup()/gnssSleep()) under /data/gnss/, sharing one filename stamp
// so they're trivially paired for PPK post-processing:
//   <stamp>.ubx  - rover side: every fully-verified UBX frame (any type,
//                  not just RAWX/SFRBX). NMEA is deliberately not captured
//                  (PPK/PPP tooling only needs the UBX stream).
//   <stamp>.rtcm - base station side: the raw RTCM3 byte stream as
//                  received from the NTRIP caster, before forwarding to the
//                  F9P (see gnssWriteRTCM()). For a network-RTK/VRS mount
//                  this is synthesized nearest the rover rather than a
//                  single physical base, but it's still the correct/only
//                  available base-side observation data for this setup,
//                  and is what PPK tooling expects regardless of how the
//                  caster produced it.
// The stamp is decided lazily, by whichever of the two streams produces
// data first each burst (usually UBX, since RTCM only starts once NTRIP
// has connected) - gnssHasDateTime() has almost always already gone true
// by then (date typically arrives before a position fix), so the stamp is
// a real timestamp in practice, not the boot-relative fallback. The actual
// File I/O for both streams happens off this task entirely - see
// sd_storage.h's SdStream API - so a wedged/degrading card (confirmed on
// live hardware: a burst of "sdmmc_send_cmd returned 0x107" errors under
// heavy burst traffic, then a call that never returned) can only ever
// block the SD writer task, never this one.
static bool ubxStreamOpenedThisBurst = false;
static bool rtcmStreamOpenedThisBurst = false;
static bool burstStampDecided = false;
static char burstStamp[24] = "";
static uint32_t gnssBurstCounter = 0;

static void gnssEnsureBurstStamp() {
  if (burstStampDecided) {
    return;
  }
  burstStampDecided = true;
  gnssBurstCounter++;
  if (gnssHasDateTime()) {
    snprintf(burstStamp, sizeof(burstStamp), "%04d%02d%02d_%02d%02d%02d_%03lu", gnssYear(), gnssMonth(),
             gnssDay(), gnssHour(), gnssMinute(), gnssSecond(), (unsigned long)(gnssBurstCounter % 1000));
  } else {
    // Rare fallback (a frame arrived before GNSS ever knew the date this
    // boot) - not collision-safe across reboots the way a real timestamp
    // is, but gnssBurstCounter at least keeps it unique within this boot.
    snprintf(burstStamp, sizeof(burstStamp), "boot_%lu_%03lu", millis(),
             (unsigned long)(gnssBurstCounter % 1000));
  }
}

static void gnssCaptureUbxFrame(const uint8_t *buf, size_t len) {
  if (!sdStorageAvailable()) {
    return;
  }
  if (!ubxStreamOpenedThisBurst) {
    ubxStreamOpenedThisBurst = true; // decide/attempt exactly once per burst, even if this open fails
    gnssEnsureBurstStamp();
    char path[64];
    snprintf(path, sizeof(path), "/data/gnss/%s.ubx", burstStamp);
    sdStreamOpen(SD_STREAM_UBX, path);
  }
  sdStreamWrite(SD_STREAM_UBX, buf, len);
}

static void gnssCaptureRtcmBytes(const uint8_t *buf, size_t len) {
  if (!sdStorageAvailable()) {
    return;
  }
  if (!rtcmStreamOpenedThisBurst) {
    rtcmStreamOpenedThisBurst = true;
    gnssEnsureBurstStamp();
    char path[64];
    snprintf(path, sizeof(path), "/data/gnss/%s.rtcm", burstStamp);
    sdStreamOpen(SD_STREAM_RTCM, path);
  }
  sdStreamWrite(SD_STREAM_RTCM, buf, len);
}

// --- UBX binary frame parser ---
// The F9P's UART interleaves NMEA (line-based, handled below) with UBX
// binary frames (0xB5 0x62 sync, class/id/length header, payload, 2-byte
// checksum). Every fully-verified frame - any type, not just RAWX/SFRBX -
// gets captured to the current burst's .ubx file below; ubxPayload is a
// separate, small, fixed snapshot of just the first bytes, kept only for
// the RXM-RTCM/ACK-NAK diagnostics that already existed here.
static uint8_t ubxState = 0;
static uint8_t ubxClass = 0, ubxMsgId = 0;
static uint16_t ubxLen = 0, ubxIdx = 0;
static uint8_t ubxPayload[16];
static uint8_t ubxCkA = 0, ubxCkB = 0;
static uint8_t ubxRxCkA = 0;

// Capacity generously covers the largest realistic frame - RXM-RAWX is
// 16 + 32*numMeas payload bytes, and a ZED-F9P tracking many signals across
// several constellations could approach ~2000. A frame that's somehow
// still bigger than this is dropped whole (not captured truncated/corrupt)
// rather than overflowing the buffer.
static const size_t UBX_CAPTURE_BUF_SIZE = 2200;
static uint8_t ubxCaptureBuf[UBX_CAPTURE_BUF_SIZE];
static size_t ubxCaptureLen = 0;
static bool ubxCaptureOverflowed = false;

static void ubxCaptureByte(uint8_t c) {
  if (ubxCaptureLen < UBX_CAPTURE_BUF_SIZE) {
    ubxCaptureBuf[ubxCaptureLen++] = c;
  } else {
    ubxCaptureOverflowed = true;
  }
}

static void ubxParseByte(uint8_t c) {
  switch (ubxState) {
    case 0:
      if (c == 0xB5) {
        ubxState = 1;
        ubxCaptureLen = 0;
        ubxCaptureOverflowed = false;
        ubxCaptureByte(c);
      }
      break;
    case 1:
      if (c == 0x62) {
        ubxState = 2;
        ubxCaptureByte(c);
      } else {
        ubxState = 0;
      }
      break;
    case 2:
      ubxClass = c;
      ubxCkA = c;
      ubxCkB = c;
      ubxState = 3;
      ubxCaptureByte(c);
      break;
    case 3:
      ubxMsgId = c;
      ubxCkA += c;
      ubxCkB += ubxCkA;
      ubxState = 4;
      ubxCaptureByte(c);
      break;
    case 4:
      ubxLen = c;
      ubxCkA += c;
      ubxCkB += ubxCkA;
      ubxState = 5;
      ubxCaptureByte(c);
      break;
    case 5:
      ubxLen |= ((uint16_t)c << 8);
      ubxCkA += c;
      ubxCkB += ubxCkA;
      ubxIdx = 0;
      ubxState = (ubxLen == 0) ? 7 : 6;
      ubxCaptureByte(c);
      break;
    case 6:
      if (ubxIdx < sizeof(ubxPayload)) {
        ubxPayload[ubxIdx] = c;
      }
      ubxIdx++;
      ubxCkA += c;
      ubxCkB += ubxCkA;
      ubxCaptureByte(c);
      if (ubxIdx >= ubxLen) {
        ubxState = 7;
      }
      break;
    case 7:
      ubxRxCkA = c;
      ubxState = 8;
      ubxCaptureByte(c);
      break;
    case 8: {
      ubxCaptureByte(c);
      bool ok = (c == ubxCkB && ubxRxCkA == ubxCkA);
      if (ok && !ubxCaptureOverflowed) {
        gnssCaptureUbxFrame(ubxCaptureBuf, ubxCaptureLen);
      }
      if (ok) {
        if (ubxClass == 0x02 && ubxMsgId == 0x32 && ubxLen >= 8) {
          // RXM-RTCM payload: version(1) flags(1) reserved/subType(2)
          // refStation(2) msgType(2). flags bit0 = crcFailed. Confirmed on
          // live hardware that RTCM validates "ok" here once real
          // corrections are flowing - only the failure case is still worth
          // surfacing. Printing every message (routinely several per
          // second under real RTCM load) was itself a Serial-blocking risk
          // stacked on top of the others fixed alongside this - same class
          // of task-watchdog exposure, just smaller.
          bool crcFailed = ubxPayload[1] & 0x01;
          if (crcFailed) {
            uint16_t refStation = (uint16_t)ubxPayload[4] | ((uint16_t)ubxPayload[5] << 8);
            uint16_t msgType = (uint16_t)ubxPayload[6] | ((uint16_t)ubxPayload[7] << 8);
            Serial.printf("[RTCM-IN] type=%u station=%u CRC-FAIL\n", msgType, refStation);
          }
        } else if (ubxClass == 0x05 && ubxMsgId == 0x00 && ubxLen >= 2) {
          // UBX-ACK-NAK only (id 0x00) - the F9P sends one of ACK/NAK for
          // every UBX-CFG-* message it receives, unconditionally. ACKs are
          // the expected/routine case once the command channel is known to
          // work; only a NAK (command rejected) is worth surfacing.
          Serial.printf("[UBX-NAK] for class=0x%02X id=0x%02X\n", ubxPayload[0], ubxPayload[1]);
        }
      }
      ubxState = 0;
      break;
    }
  }
}

void gnssEnableGST() {
  GPSSerial.write(UBX_ENABLE_GST_UART1, sizeof(UBX_ENABLE_GST_UART1));
}

void gnssEnableRxmRtcmDiag() {
  GPSSerial.write(UBX_ENABLE_RXM_RTCM_UART1, sizeof(UBX_ENABLE_RXM_RTCM_UART1));
}

void gnssEnableRtcm3Input() {
  GPSSerial.write(UBX_SET_UART1_INPROT_RTCM3X, sizeof(UBX_SET_UART1_INPROT_RTCM3X));
}

// Safe to call once per boot only - HardwareSerial::setRxBufferSize()
// throws an error if the UART is already running. The persistent main
// loop calls initGNSS() once at startup; each subsequent wake cycle only
// needs the message-enable resend below (backup mode can reset it), not a
// fresh UART begin(), so that part is split into gnssReenableMessages().
static bool uartStarted = false;

void gnssReenableMessages() {
  gnssEnableGST();
  gnssEnableRtcm3Input();
  GPSSerial.write(UBX_ENABLE_RXM_RTCM_UART1, sizeof(UBX_ENABLE_RXM_RTCM_UART1));

  if (sdStorageAvailable()) {
    GPSSerial.write(UBX_ENABLE_RAWX_UART1, sizeof(UBX_ENABLE_RAWX_UART1));
    GPSSerial.write(UBX_ENABLE_SFRBX_UART1, sizeof(UBX_ENABLE_SFRBX_UART1));
  }
}

void initGNSS() {
  if (!uartStarted) {
    GPSSerial.setRxBufferSize(1024);
    GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    uartStarted = true;
    delay(100);
  }

  gnssReenableMessages();
}

void gnssSleep() {
  GPSSerial.write(UBX_PMREQ_BACKUP_EXTINT, sizeof(UBX_PMREQ_BACKUP_EXTINT));
  GPSSerial.flush();

  // End of this burst - close both capture streams (a stream never opened
  // this burst, e.g. no RTCM if NTRIP never connected, just no-ops on the
  // writer task) so the next burst starts fresh ones.
  sdStreamClose(SD_STREAM_UBX);
  sdStreamClose(SD_STREAM_RTCM);
}

void gnssWakeup() {
  // Start of a new burst - reset capture state so the next captured
  // UBX/RTCM byte decides a fresh stamp/file. Defensive close() in case a
  // previous burst somehow never got to gnssSleep() (e.g. skipped this
  // cycle).
  sdStreamClose(SD_STREAM_UBX);
  sdStreamClose(SD_STREAM_RTCM);
  ubxStreamOpenedThisBurst = false;
  rtcmStreamOpenedThisBurst = false;
  burstStampDecided = false;

  // Wake trigger is an edge on EXTINT (u-blox docs/reference drivers note
  // it can be a falling or rising edge) - drive both to not depend on which.
  pinMode(GPS_EXTINT_PIN, OUTPUT);
  digitalWrite(GPS_EXTINT_PIN, LOW);
  delay(10);
  digitalWrite(GPS_EXTINT_PIN, HIGH);
  delay(10);
  digitalWrite(GPS_EXTINT_PIN, LOW);
}

void pollGNSS() {
  unsigned long nowMs = millis();
  bool freshData = nowMs - lastPollGnssMs < ANCHOR_MAX_POLL_GAP_MS;
  lastPollGnssMs = nowMs;

  while (GPSSerial.available() > 0) {
    char c = GPSSerial.read();
    if (DEBUG_PRINT_RAW_GNSS) {
      Serial.write(c);
    }
    gps.encode(c);
    ubxParseByte((uint8_t)c); // captures completed UBX frames to the current burst's .ubx file itself

    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      if (lineLen >= 6 && strncmp(lineBuf + 3, "GGA", 3) == 0) {
        strncpy(lastGGA, lineBuf, sizeof(lastGGA) - 1);
        lastGGA[sizeof(lastGGA) - 1] = '\0';
        haveGGA = true;
      }
      lineLen = 0;
    } else if (c != '\r' && lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }

  if (freshData) {
    maybeUpdateEpochAnchor();
  }
}

void gnssWriteRTCM(const uint8_t *buf, size_t len) {
  gnssCaptureRtcmBytes(buf, len); // base-station side of the PPK pair - see the capture block above
  GPSSerial.write(buf, len);
}

bool gnssHasFix() {
  return gps.location.isValid();
}

int gnssFixQuality() {
  return ggaFixQuality.isValid() ? atoi(ggaFixQuality.value()) : 0;
}

const char *gnssFixQualityLabel(int quality) {
  switch (quality) {
    case 0: return "no fix";
    case 1: return "autonomous";
    case 2: return "DGPS";
    case 3: return "PPS";
    case 4: return "RTK FIXED";
    case 5: return "RTK FLOAT";
    case 6: return "estimated/dead-reckoning";
    default: return "unknown";
  }
}

bool gnssHasAccuracyEstimate() {
  return gstStdLat.isValid();
}

float gnssHorizontalAccuracy() {
  float latErr = atof(gstStdLat.value());
  float lonErr = atof(gstStdLon.value());
  return sqrt(latErr * latErr + lonErr * lonErr);
}

float gnssVerticalAccuracy() {
  return atof(gstStdAlt.value());
}

double gnssLatitude() { return gps.location.lat(); }
double gnssLongitude() { return gps.location.lng(); }
double gnssAltitude() { return gps.altitude.meters(); }

uint32_t gnssSatellites() {
  return gps.satellites.isValid() ? gps.satellites.value() : 0;
}

bool gnssHasDateTime() {
  return gps.date.isValid() && gps.time.isValid();
}

void gnssTimestamp(char *buf, size_t bufSize) {
  snprintf(buf, bufSize, "%04d-%02d-%02dT%02d:%02d:%02dZ",
           gps.date.year(), gps.date.month(), gps.date.day(),
           gps.time.hour(), gps.time.minute(), gps.time.second());
}

void gnssDateString(char *buf, size_t bufSize) {
  snprintf(buf, bufSize, "%04d%02d%02d", gps.date.year(), gps.date.month(), gps.date.day());
}

uint16_t gnssYear() { return gps.date.year(); }
uint8_t gnssMonth() { return gps.date.month(); }
uint8_t gnssDay() { return gps.date.day(); }
uint8_t gnssHour() { return gps.time.hour(); }
uint8_t gnssMinute() { return gps.time.minute(); }
uint8_t gnssSecond() { return gps.time.second(); }
uint16_t gnssMillisecond() { return gps.time.centisecond() * 10; }

// Howard Hinnant's days_from_civil algorithm (public domain) - avoids any
// mktime()/timezone dependency for a straightforward UTC-to-epoch
// conversion.
static int64_t daysFromCivilUTC(int y, unsigned m, unsigned d) {
  y -= m <= 2;
  int64_t era = (y >= 0 ? y : y - 399) / 400;
  unsigned yoe = (unsigned)(y - era * 400);
  unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + (int64_t)doe - 719468;
}

uint64_t gnssEpochMillis() {
  int64_t days = daysFromCivilUTC(gnssYear(), gnssMonth(), gnssDay());
  uint64_t secs =
      (uint64_t)days * 86400ULL + gnssHour() * 3600ULL + gnssMinute() * 60ULL + gnssSecond();
  return secs * 1000ULL + gnssMillisecond();
}

// Only re-anchors when the GNSS-reported wall clock has actually advanced
// since the last call - re-anchoring unconditionally on every pollGNSS()
// call (including the many calls during idle where the F9P has nothing new
// to report at all) would just keep resetting epochAnchorAtMillis against
// the same stale fields, which cancels out the millis() extrapolation this
// exists to provide in the first place.
static void maybeUpdateEpochAnchor() {
  if (!gnssHasDateTime()) {
    return;
  }
  uint16_t y = gnssYear();
  uint8_t mo = gnssMonth(), d = gnssDay(), h = gnssHour(), mi = gnssMinute(), se = gnssSecond();
  if (y == lastAnchoredYear && mo == lastAnchoredMonth && d == lastAnchoredDay &&
      h == lastAnchoredHour && mi == lastAnchoredMinute && se == lastAnchoredSecond) {
    return;
  }
  lastAnchoredYear = y;
  lastAnchoredMonth = mo;
  lastAnchoredDay = d;
  lastAnchoredHour = h;
  lastAnchoredMinute = mi;
  lastAnchoredSecond = se;
  uint64_t anchorMs = gnssEpochMillis();
  unsigned long atMillis = millis();
  portENTER_CRITICAL(&anchorMux);
  epochAnchorMs = anchorMs;
  epochAnchorAtMillis = atMillis;
  haveEpochAnchor = true;
  portEXIT_CRITICAL(&anchorMux);
}

bool gnssHasEpochAnchor() {
  return haveEpochAnchor;
}

// Unlike gnssEpochMillis() (a snapshot of TinyGPS++'s last-received, never-
// expiring fields - frozen for as long as the F9P goes without reporting,
// e.g. all through backup-mode sleep), this keeps advancing via millis()
// extrapolation off the last genuinely fresh fix. Use this, not
// gnssYear()/gnssHour()/etc. directly, for anything that needs "the actual
// current time" spanning an idle period - see the comment on the anchor
// statics above for what broke when miniseed_writer.cpp used the frozen
// version.
uint64_t gnssEpochMillisNow() {
  if (!haveEpochAnchor) {
    return 0;
  }
  portENTER_CRITICAL(&anchorMux);
  uint64_t anchorMs = epochAnchorMs;
  unsigned long atMillis = epochAnchorAtMillis;
  portEXIT_CRITICAL(&anchorMux);
  return anchorMs + (uint64_t)(millis() - atMillis);
}

// Howard Hinnant's civil_from_days algorithm (public domain) - inverse of
// daysFromCivilUTC() above.
static void civilFromDaysUTC(int64_t z, int *y, unsigned *m, unsigned *d) {
  z += 719468;
  int64_t era = (z >= 0 ? z : z - 146096) / 146097;
  unsigned doe = (unsigned)(z - era * 146097);                         // [0, 146096]
  unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365; // [0, 399]
  int64_t yy = (int64_t)yoe + era * 400;
  unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100); // [0, 365]
  unsigned mp = (5 * doy + 2) / 153;                      // [0, 11]
  unsigned dd = doy - (153 * mp + 2) / 5 + 1;             // [1, 31]
  unsigned mm = mp < 10 ? mp + 3 : mp - 9;                // [1, 12]
  *y = (int)(yy + (mm <= 2 ? 1 : 0));
  *m = mm;
  *d = dd;
}

// Broken-down UTC calendar fields for gnssEpochMillisNow() - what
// miniseed_writer.cpp actually needs (SEED's BTIME format), without every
// caller having to redo the epoch-ms -> civil-date conversion itself.
void gnssCivilNow(uint16_t *year, uint8_t *month, uint8_t *day, uint8_t *hour, uint8_t *minute,
                   uint8_t *second, uint16_t *milli) {
  gnssCivilFromEpochMillis(gnssEpochMillisNow(), year, month, day, hour, minute, second, milli);
}

void gnssCivilFromEpochMillis(uint64_t ms, uint16_t *year, uint8_t *month, uint8_t *day,
                              uint8_t *hour, uint8_t *minute, uint8_t *second, uint16_t *milli) {
  uint64_t totalSecs = ms / 1000ULL;
  *milli = (uint16_t)(ms % 1000ULL);
  int64_t days = (int64_t)(totalSecs / 86400ULL);
  uint32_t secOfDay = (uint32_t)(totalSecs % 86400ULL);
  *hour = (uint8_t)(secOfDay / 3600);
  *minute = (uint8_t)((secOfDay % 3600) / 60);
  *second = (uint8_t)(secOfDay % 60);
  int y;
  unsigned m, d;
  civilFromDaysUTC(days, &y, &m, &d);
  *year = (uint16_t)y;
  *month = (uint8_t)m;
  *day = (uint8_t)d;
}

const char *gnssLastGGA() {
  return haveGGA ? lastGGA : "";
}

uint32_t gnssCharsProcessed() {
  return gps.charsProcessed();
}
