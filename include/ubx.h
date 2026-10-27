#pragma once
#include <stddef.h>
#include <stdint.h>

// UBX frame building - pure, header-only, unit-tested on the host
// (test/test_native) so frames don't depend on hand-computed checksums.

// 8-bit Fletcher checksum over class, id, length and payload (i.e. the
// frame minus its two sync chars and two checksum bytes).
inline void ubxChecksum(const uint8_t *data, size_t len, uint8_t *ckA, uint8_t *ckB) {
  uint8_t a = 0, b = 0;
  for (size_t i = 0; i < len; i++) {
    a += data[i];
    b += a;
  }
  *ckA = a;
  *ckB = b;
}

// UBX-CFG-MSG (class 0x06, id 0x01), 8-byte payload: sets the output rate
// of message msgClass/msgId on UART1 only (all other ports 0). out must
// hold UBX_CFG_MSG_FRAME_LEN bytes.
static const size_t UBX_CFG_MSG_FRAME_LEN = 16;
inline void ubxBuildCfgMsgUart1(uint8_t msgClass, uint8_t msgId, uint8_t uart1Rate, uint8_t *out) {
  const uint8_t frame[UBX_CFG_MSG_FRAME_LEN] = {
      0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
      msgClass, msgId,
      0x00, uart1Rate, 0x00, 0x00, 0x00, 0x00, // rates: DDC, UART1, UART2, USB, SPI, reserved
      0x00, 0x00};
  for (size_t i = 0; i < UBX_CFG_MSG_FRAME_LEN; i++) {
    out[i] = frame[i];
  }
  ubxChecksum(out + 2, UBX_CFG_MSG_FRAME_LEN - 4, &out[14], &out[15]);
}
