// ============================================================
//  핀 설정 — 보드별 분기 (S3 ↔ C3 이식은 이 파일과 platformio.ini 환경만 바꾸면 됨)
// ============================================================
#pragma once

#define GpsSerial     Serial1      // 두 보드 모두 UART1 사용 (핀은 GPIO 매트릭스로 자유 배정)

#if CONFIG_IDF_TARGET_ESP32C3
// ------------------------------------------------------------
//  ESP32-C3 (C3-MINI-1 / Super Mini)
//  ※ GPIO18/19 = USB, GPIO2/8/9 = 스트래핑(8은 Super Mini 내장 LED, 9는 BOOT 버튼) → 피할 것
//  ※ GPIO20/21은 UART0 기본 핀이지만 콘솔을 USB-CDC로 쓰므로 비어 있음
// ------------------------------------------------------------
#define PIN_GPS_RX    20   // ESP32 RX  <- GPS TXD
#define PIN_GPS_TX    21   // ESP32 TX  -> GPS RXD
#define PIN_IMU_SCK   4    // 모듈 실크 SCL/SCLK   (C3 기본 SPI 핀 배치)
#define PIN_IMU_MISO  5    // 모듈 실크 AD0/MISO
#define PIN_IMU_MOSI  6    // 모듈 실크 SDA/MOSI
#define PIN_IMU_CS    7    // 모듈 실크 CS

#else
// ------------------------------------------------------------
//  ESP32-S3-WROOM-1 N16R8 (현재 실기기)
//  ※ GPIO35/36/37 = OPI PSRAM 전용(사용 금지), GPIO19/20 = USB, GPIO0/3/45/46 = 스트래핑 핀(피할 것)
// ------------------------------------------------------------
// ---- ATGM336H GPS (UART1, 3.3V, 기본 9600bps → 부팅 시 115200/10Hz로 전환) ----
//  부팅 시 두 핀·두 보레이트를 자동 탐색하므로 TX/RX가 뒤바뀌어도 동작한다(시리얼 로그에 표시).
#define PIN_GPS_RX    18   // ESP32 RX  <- GPS TXD
#define PIN_GPS_TX    17   // ESP32 TX  -> GPS RXD

// ---- ICM-42688-P 모듈 (보라색 브레이크아웃, SPI 모드 0, 3.3V) ----
//  S3 기본 FSPI 핀. 모듈 VCC → 3V3, GND → GND. 3V3/NC/INT1/INT2 미연결
#define PIN_IMU_SCK   12   // 모듈 실크 SCL/SCLK
#define PIN_IMU_MISO  13   // 모듈 실크 AD0/MISO (칩 SDO)
#define PIN_IMU_MOSI  11   // 모듈 실크 SDA/MOSI (칩 SDI)
#define PIN_IMU_CS    10   // 모듈 실크 CS (LOW 활성, HIGH면 I2C 모드)
#endif
