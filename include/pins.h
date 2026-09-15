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

// ---- ICM-42688-P 모듈 (보라색 브레이크아웃, SPI 모드 0, 3.3V) ----
//  S3 기본 FSPI 핀. 모듈 VCC → 3V3, GND → GND. 3V3/NC/INT1/INT2 미연결
#define PIN_IMU_SCK   12   // 모듈 실크 SCL/SCLK
#define PIN_IMU_MISO  13   // 모듈 실크 AD0/MISO (칩 SDO)
#define PIN_IMU_MOSI  11   // 모듈 실크 SDA/MOSI (칩 SDI)
#define PIN_IMU_CS    10   // 모듈 실크 CS (LOW 활성, HIGH면 I2C 모드)
