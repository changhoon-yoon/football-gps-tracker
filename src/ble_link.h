// ============================================================
//  BLE 링크 (NimBLE) — 폰 브라우저(Web Bluetooth) / 앱으로 실시간 전송
//
//  서비스  f007ba11-0000-4c45-8000-000000000000
//   FIX    ...-0001  notify 10Hz, 20B  (아래 프레임 규격)
//   STATS  ...-0002  notify 1Hz,  22B
//   HIST   ...-0003  notify, 저장 동선 재생: 점당 8B(lat_e7,lon_e7) × N, 끝 마커 = lat 0x7FFFFFFF + 총점수
//   CTRL   ...-0004  write 1B: 0x01 리셋, 0x02 이력 재요청
//
//  FIX (little-endian, 20B)
//   0  u8  flags  bit0 fix, bit1 gps alive, bit2 imu ok, bit3 점 저장됨, bit4 세션 시작
//   1  u8  sats
//   2  u8  hdop×10
//   3  u8  bit0-2 zone 0~4, bit3-7 보이는 위성 수(GSV 합, 최대 31) — fix 전 안테나 상태 표시용
//   4  i32 lat ×1e7
//   8  i32 lon ×1e7
//   12 u16 speed cm/s
//   14 u16 course ×10
//   16 u32 dist cm
//  STATS (22B)
//   0  u16 max km/h ×10   2 u16 elapsed s   4..13 u16 zone[5] m   14 u16 sprints   16 u16 PL×10   18 u16 points   20 i16 고도 m
// ============================================================
#pragma once
#include <Arduino.h>
#include "track.h"

static inline void putU16(uint8_t* p, uint16_t v) { p[0] = v & 0xFF; p[1] = v >> 8; }
static inline void putU32(uint8_t* p, uint32_t v) { p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; p[2] = (v >> 16) & 0xFF; p[3] = v >> 24; }
static inline uint16_t clampU16(float v) { return v < 0 ? 0 : v > 65535.0f ? 65535 : (uint16_t)v; }
static inline uint8_t  clampU8(float v)  { return v < 0 ? 0 : v > 255.0f ? 255 : (uint8_t)v; }

void bleSetup(const char* name, TrackStore* track, void (*onReset)());
void bleLoop();                                   // 이력 스트리밍 진행 (loop에서 호출)
void bleNotifyFix(const uint8_t* frame, size_t len);
void bleNotifyStats(const uint8_t* frame, size_t len);
bool bleConnected();
uint16_t bleMtu();
