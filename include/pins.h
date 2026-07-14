#pragma once

// Centralized pin/hardware configuration for the ESP32 DevKit V1
// + ZED-F9P (gnss.store ELT0128), UART1 on the F9P (SEL jumper unsoldered).
//   F9P UART1 TXO -> ESP32 GPIO16 (RX2)
//   F9P UART1 RXI -> ESP32 GPIO17 (TX2)
//   F9P VCC       -> 3.3V (or 5V pin if using external 5V supply)
//   F9P GND       -> GND

#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GPS_BAUD 38400
