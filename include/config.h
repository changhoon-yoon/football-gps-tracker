// ============================================================
//  동작 파라미터 — 지표 계산·네트워크·버퍼 튜닝은 여기서
// ============================================================
#pragma once

// ---- 무선 선택 ----
//  ENABLE_WIFI 1 + ENABLE_BLE 1 : 둘 다 (책상 테스트). BLE만 쓰면 WiFi를 0으로 → 배터리 2~3배
#ifndef ENABLE_WIFI               // platformio.ini의 -D로 보드별 덮어쓰기 가능
#define ENABLE_WIFI     1
#endif
#ifndef ENABLE_BLE
#define ENABLE_BLE      1
#endif
#define BLE_NAME        "FootTrack"       // 폰 Web Bluetooth 기기 선택창에 보이는 이름

// ---- 네트워크 ----
#define HOSTNAME        "foottrack"       // mDNS: http://foottrack.local (iOS OK, Android은 IP로)
#define AP_SSID         "FootTrack"       // 핫스팟 접속 실패 시 자체 AP → http://192.168.4.1
#define AP_PASS         "foottrack1234"   // 8자 이상
#define STA_TIMEOUT_MS  15000             // 핫스팟 접속 대기

// ---- GPS ----
#define GPS_BAUD_INIT   9600              // ATGM336H 공장 기본
#define GPS_BAUD_RUN    115200            // 10Hz × (GGA+RMC)는 9600으로 부족
#define GPS_NAV_MODE    2                 // PCAS11 항법 모드: 0 휴대, 1 정지, 2 보행자, 3 차량(공장값, 저속을 0으로 눌러 걷기 거리 누락)
#define GPS_MIN_SATS    4
#define GPS_MAX_HDOP    6.0f
#define GPS_MAX_AGE_MS  1500

// ---- 거리/속도 (FIFA·Catapult 관례) ----
#define MIN_SPEED_MPS   0.5f   // 이하면 정지로 간주 — Doppler 속도 지터 제거 (1.8 km/h)
#define MAX_GAP_S       2.0f   // 이보다 긴 fix 공백은 거리 적분에서 제외
#define ZONE_KMH_1      7.0f   // Z1 걷기      < 7
#define ZONE_KMH_2     14.0f   // Z2 조깅      7~14
#define ZONE_KMH_3     20.0f   // Z3 러닝     14~20
#define ZONE_KMH_4     25.0f   // Z4 고속러닝 20~25, Z5 스프린트 > 25
#define SPRINT_MIN_MS   1000   // Z5를 이 시간 이상 유지해야 스프린트 1회

// ---- 트랙 버퍼 (점 1개 = 12B) ----
#define TRACK_MIN_STEP_M 1.0f  // 마지막 저장점에서 이만큼 벗어나야 새 점 저장 (정지 지터 블롭 방지)
#ifndef TRACK_CAP_SRAM
#define TRACK_CAP_SRAM   6000  // PSRAM 없을 때 (72KB). C3 환경은 platformio.ini에서 3000으로 줄임
#endif
#define TRACK_CAP_PSRAM  60000 // PSRAM 있을 때 (720KB) — 가득 차면 절반으로 솎아내며 계속 기록

// ---- IMU ----
#define IMU_RATE_HZ     100
