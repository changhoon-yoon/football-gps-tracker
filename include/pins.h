// ============================================================
//  핀 설정 — ESP32-S3-WROOM-1 N16R8
//  ※ GPIO35/36/37 = OPI PSRAM 전용(사용 금지), GPIO19/20 = USB,
//     GPIO0/3/45/46 = 스트래핑 핀(피할 것)
// ============================================================
#pragma once

// ---- ATGM336H GPS (UART1, 3.3V, 기본 9600bps → 부팅 시 115200/10Hz로 전환) ----
#define PIN_GPS_RX    18   // ESP32 RX  <- GPS TXD
#define PIN_GPS_TX    17   // ESP32 TX  -> GPS RXD
#define GpsSerial     Serial1

// ---- ICM-42688-P IMU (SPI 모드 0, 3.3V) ----
//  S3 기본 FSPI 핀. VDDIO/VDD → 3V3, GND → GND
#define PIN_IMU_SCK   12   // SCLK
#define PIN_IMU_MISO  13   // SDO / AD0
#define PIN_IMU_MOSI  11   // SDI / SDA
#define PIN_IMU_CS    10   // CS (LOW 활성)
