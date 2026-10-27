#include "geophone_parse.h"
#include <string.h>

// Keys only ever appear once per object, and no string *value* in this
// protocol can contain a quoted key (sample values are bare hex digits), so
// a plain substring search for the quoted key is unambiguous here.
static const char *findValue(const char *json, size_t len, const char *quotedKey) {
  size_t keyLen = strlen(quotedKey);
  if (len < keyLen) {
    return nullptr;
  }
  for (size_t i = 0; i + keyLen <= len; i++) {
    if (memcmp(json + i, quotedKey, keyLen) == 0) {
      const char *p = json + i + keyLen;
      const char *end = json + len;
      while (p < end && (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')) {
        p++;
      }
      if (p >= end || *p != ':') {
        return nullptr;
      }
      p++;
      while (p < end && (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')) {
        p++;
      }
      return p < end ? p : nullptr;
    }
  }
  return nullptr;
}

static bool parseUInt(const char *p, const char *end, uint32_t *out) {
  if (p >= end || *p < '0' || *p > '9') {
    return false;
  }
  uint64_t v = 0;
  while (p < end && *p >= '0' && *p <= '9') {
    v = v * 10 + (uint64_t)(*p - '0');
    if (v > 0xFFFFFFFFULL) {
      return false;
    }
    p++;
  }
  *out = (uint32_t)v;
  return true;
}

static int hexDigit(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

bool geophoneParsePacket(const char *json, size_t len, GeophonePacket *out) {
  const char *end = json + len;
  out->isChannelData = false;
  out->isMarker = false;
  out->channel[0] = '\0';
  out->sampleIntervalUs = 0;
  out->count = 0;
  out->msec = 0;

  const char *cn = findValue(json, len, "\"CN\"");
  if (cn == nullptr) {
    const char *ms = findValue(json, len, "\"MSEC\"");
    if (ms == nullptr || !parseUInt(ms, end, &out->msec)) {
      return false;
    }
    out->isMarker = true;
    return true;
  }

  // CN: exactly 3 characters inside quotes.
  if (cn + 5 > end || cn[0] != '"' || cn[4] != '"') {
    return false;
  }
  memcpy(out->channel, cn + 1, 3);
  out->channel[3] = '\0';

  const char *si = findValue(json, len, "\"SI\"");
  if (si != nullptr) {
    parseUInt(si, end, &out->sampleIntervalUs);
  }

  const char *p = findValue(json, len, "\"DS\"");
  if (p == nullptr || *p != '[') {
    return false;
  }
  p++;

  // DS entries are signed 32-bit values printed as unpadded hex, so a
  // negative one is always all 8 digits ("FFFFA804"): accumulate unsigned
  // and reinterpret, never parse as signed (see raspberry_shake.md).
  for (;;) {
    while (p < end && (*p == ' ' || *p == ',' || *p == '\t' || *p == '\r' || *p == '\n')) {
      p++;
    }
    if (p >= end) {
      return false; // truncated before the closing ']'
    }
    if (*p == ']') {
      break;
    }
    if (*p != '"') {
      return false;
    }
    p++;
    uint32_t v = 0;
    int digits = 0;
    while (p < end && *p != '"') {
      int d = hexDigit(*p);
      if (d < 0 || ++digits > 8) {
        return false;
      }
      v = (v << 4) | (uint32_t)d;
      p++;
    }
    if (p >= end || digits == 0) {
      return false;
    }
    p++; // closing quote
    if (out->count >= GEOPHONE_MAX_SAMPLES) {
      return false;
    }
    out->samples[out->count++] = (int32_t)v;
  }

  out->isChannelData = true;
  return true;
}
