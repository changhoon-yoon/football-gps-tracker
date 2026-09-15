// ============================================================
//  FootTrack — 축구/풋살 실시간 동선 트래커 펌웨어
//  ESP32-S3-WROOM-1 + ATGM336H(GPS 10Hz) + ICM-42688-P(IMU 100Hz)
//
//  동작
//   1. 부팅 시 폰 핫스팟(STA) 접속 시도 → 실패하면 자체 AP(192.168.4.1)
//   2. GPS를 115200bps / 10Hz / GGA+RMC만 출력하도록 설정
//   3. RMC(fix) 틱마다 지표 갱신 → SSE(/events)로 폰 브라우저에 푸시
//   4. 웹페이지(/)가 Leaflet 지도 위에 동선을 실시간으로 그림
//
//  거리 계산: GPS 위치 차분이 아니라 Doppler 속도(RMC) 적분 + 정지 임계값.
//            위치 차분은 정지 상태의 좌표 지터가 거리로 누적되므로 쓰지 않는다.
// ============================================================
#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <TinyGPSPlus.h>
#include <time.h>
#include <math.h>
#include <memory>

#include "pins.h"
#include "config.h"
#include "track.h"
#include "imu_icm42688.h"
#include "web_page.h"
#include "ble_link.h"

#if __has_include("wifi_secrets.h")
  #include "wifi_secrets.h"
#else
  #warning "include/wifi_secrets.h 없음 - AP 모드로만 동작 (wifi_secrets.h.example 참고)"
  #define WIFI_SSID ""
  #define WIFI_PASS ""
#endif

// ---------------- 전역 ----------------
TinyGPSPlus      gps;
// GSV 3번 필드 = 해당 위성계에서 "보이는" 위성 수. fix 전 안테나 상태 피드백용 (GPS / BeiDou 신구 표기 / GLONASS / Galileo)
TinyGPSCustom    gsvGP(gps, "GPGSV", 3), gsvGB(gps, "GBGSV", 3), gsvBD(gps, "BDGSV", 3),
                 gsvGL(gps, "GLGSV", 3), gsvGA(gps, "GAGSV", 3);
ICM42688         imu;
TrackStore       track;
AsyncWebServer   server(80);
AsyncEventSource events("/events");

struct Session {
  bool     started = false;       // 첫 유효 fix에서 시작
  uint32_t start_ms = 0;
  uint32_t start_epoch = 0;       // GPS UTC epoch (GPX 타임스탬프용), 0이면 미상
  uint32_t last_fix_ms = 0;
  // 현재값
  bool     fix = false;
  double   lat = 0, lon = 0;
  float    kmh = 0, course = 0;
  // 누적 지표
  float    dist_m = 0;
  float    max_kmh = 0;
  float    zone_m[5] = {0, 0, 0, 0, 0};
  uint16_t sprints = 0;
  bool     in_sprint = false, sprint_counted = false;
  uint32_t sprint_start_ms = 0;
  float    player_load = 0;
  // 트랙 저장 상태
  double   last_pt_lat = 0, last_pt_lon = 0;
  float    track_step_m = TRACK_MIN_STEP_M;
};
static Session S;

static bool     g_wifi_sta = false;
static uint32_t g_last_gps_byte_ms = 0;
static char     g_nmea_line[96];     // 진단용: 마지막 완성 NMEA 문장
static char     g_nmea_buf[96];
static uint8_t  g_nmea_len = 0;
static uint32_t g_nmea_count = 0;
static uint32_t g_last_push_ms = 0;
static char     g_json[600];

// ============================================================
//  GPS (ATGM336H, CASIC PCAS 명령)
// ============================================================
static void gpsSend(const char* body) {
  uint8_t cs = 0;
  for (const char* p = body; *p; ++p) cs ^= (uint8_t)*p;
  GpsSerial.printf("$%s*%02X\r\n", body, cs);
  GpsSerial.flush();
  delay(60);
}

static int  g_gps_rx = PIN_GPS_RX, g_gps_tx = PIN_GPS_TX;
static bool g_gps_found = false;

// rxPin에서만 듣고(TX 미할당 → 충돌 없음) NMEA 문장 머리('$'+대문자) 개수를 센다
static uint32_t gpsListen(int rxPin, uint32_t baud, uint32_t ms) {
  GpsSerial.end();
  pinMode(PIN_GPS_RX, INPUT);
  pinMode(PIN_GPS_TX, INPUT);
  GpsSerial.begin(baud, SERIAL_8N1, rxPin, -1);
  uint32_t nmea = 0; int prev = 0;
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    while (GpsSerial.available()) {
      const int c = GpsSerial.read();
      if (prev == '$' && c >= 'A' && c <= 'Z') nmea++;
      prev = c;
    }
    delay(2);
  }
  GpsSerial.end();
  return nmea;
}

// 감지된 핀/보레이트로 열고 115200 / 10Hz / GGA+RMC 설정
static void gpsConfigure(uint32_t currentBaud) {
  GpsSerial.setRxBufferSize(2048);
  GpsSerial.begin(currentBaud, SERIAL_8N1, g_gps_rx, g_gps_tx);
  delay(50);
  if (currentBaud != GPS_BAUD_RUN) {
    gpsSend("PCAS01,5");                        // 5 = 115200bps
    GpsSerial.updateBaudRate(GPS_BAUD_RUN);
    delay(100);
  }
  gpsSend("PCAS03,1,0,0,1,1,0,0,0,0,0,,,0,0");  // GGA=1, GSV=1(보이는 위성 수), RMC=1, 나머지 0
  gpsSend("PCAS02,100");                        // 100ms 주기 = 10Hz (최대)
}

// 배선 TX/RX 뒤바뀜과 보레이트 잔존(전원 유지 시 115200)을 자동 감지. 최대 약 5초.
static bool gpsProbe() {
  const int      pins[2]  = {PIN_GPS_RX, PIN_GPS_TX};
  const uint32_t bauds[2] = {GPS_BAUD_RUN, GPS_BAUD_INIT};
  for (int p = 0; p < 2; p++) {
    for (int b = 0; b < 2; b++) {
      const uint32_t n = gpsListen(pins[p], bauds[b], 1300);
      if (n < 2) continue;
      g_gps_rx = pins[p];
      g_gps_tx = (pins[p] == PIN_GPS_RX) ? PIN_GPS_TX : PIN_GPS_RX;
      Serial.printf("[GPS] 감지: GPIO%d에서 NMEA %lu문장 @%lu bps%s\n", g_gps_rx, (unsigned long)n,
                    (unsigned long)bauds[b], p ? "  ※ pins.h와 TX/RX 반대 — 자동 적용" : "");
      gpsConfigure(bauds[b]);
      Serial.printf("[GPS] RX=GPIO%d TX=GPIO%d, %d bps, 10Hz, GGA+RMC\n", g_gps_rx, g_gps_tx, GPS_BAUD_RUN);
      return true;
    }
  }
  Serial.println("[GPS] 신호 없음 — GPIO18/17 양쪽, 9600/115200 모두 조용함. 모듈 VCC/GND/TXD 확인 (15초 후 재시도)");
  // 라인 상태 진단: 살아있는 UART TX는 idle HIGH를 스스로 유지한다.
  //  PD/PU 양쪽 모두 HIGH = 모듈이 구동(정상 TXD) / 양쪽 LOW = LOW로 눌림 / PD:LOW·PU:HIGH = 플로팅(미연결·모듈 정지)
  for (int p = 0; p < 2; p++) {
    pinMode(pins[p], INPUT_PULLDOWN); delay(5); const int pd = digitalRead(pins[p]);
    pinMode(pins[p], INPUT_PULLUP);   delay(5); const int pu = digitalRead(pins[p]);
    pinMode(pins[p], INPUT);
    const char* verdict = (pd && pu) ? "구동 HIGH (살아있는 TXD 후보)" : (!pd && !pu) ? "LOW로 눌림" : "플로팅 (미연결 또는 모듈 정지)";
    Serial.printf("[GPS]   GPIO%d 라인: PD=%d PU=%d → %s\n", pins[p], pd, pu, verdict);
  }
  GpsSerial.setRxBufferSize(2048);
  GpsSerial.begin(GPS_BAUD_RUN, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  return false;
}

// 보이는 위성 수 합계 (3초 내 갱신된 위성계만)
static uint8_t satsInView() {
  TinyGPSCustom* all[] = {&gsvGP, &gsvGB, &gsvBD, &gsvGL, &gsvGA};
  int n = 0;
  for (TinyGPSCustom* c : all) if (c->isValid() && c->age() < 3000) n += atoi(c->value());
  return n > 255 ? 255 : (uint8_t)n;
}

// Howard Hinnant days_from_civil — TZ 의존 없이 UTC epoch 계산
static uint32_t civilToEpoch(int y, unsigned m, unsigned d, unsigned hh, unsigned mm, unsigned ss) {
  y -= m <= 2;
  const int era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = (unsigned)(y - era * 400);
  const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  const int64_t days = (int64_t)era * 146097 + (int64_t)doe - 719468;
  return (uint32_t)(days * 86400 + hh * 3600 + mm * 60 + ss);
}

static uint32_t gpsEpoch() {
  if (!gps.date.isValid() || !gps.time.isValid() || gps.date.year() < 2020) return 0;
  return civilToEpoch(gps.date.year(), gps.date.month(), gps.date.day(),
                      gps.time.hour(), gps.time.minute(), gps.time.second());
}

// ============================================================
//  지표 계산
// ============================================================
static void resetSession() {
  track.clear();
  S = Session();
  Serial.println("[SESSION] 리셋");
}

static void pushStatus(bool stored);

// RMC(fix 있는 문장) 1개 = 1틱 (10Hz)
static void onGpsTick() {
  const uint32_t now = millis();
  const float    v_raw = gps.speed.mps();                   // value() 호출로 updated 플래그 소거
  const bool     hdopOk = !gps.hdop.isValid() || gps.hdop.hdop() <= GPS_MAX_HDOP;
  const bool     valid = gps.location.isValid() && gps.location.age() < GPS_MAX_AGE_MS
                      && gps.satellites.value() >= GPS_MIN_SATS && hdopOk;
  bool stored = false;
  S.fix = valid;

  if (valid) {
    if (!S.started) {
      S.started = true;
      S.start_ms = now;
      S.last_fix_ms = now;
      S.start_epoch = gpsEpoch();
      Serial.printf("[SESSION] 시작 (UTC epoch %lu)\n", (unsigned long)S.start_epoch);
    }
    float v = v_raw < MIN_SPEED_MPS ? 0.0f : v_raw;          // 정지 지터 제거
    float dt = (now - S.last_fix_ms) / 1000.0f;
    S.last_fix_ms = now;
    if (dt > MAX_GAP_S) dt = 0;                              // 긴 공백은 적분 제외

    const float d   = v * dt;
    const float kmh = v * 3.6f;
    S.lat = gps.location.lat();
    S.lon = gps.location.lng();
    S.kmh = kmh;
    S.course = gps.course.deg();
    S.dist_m += d;
    if (kmh > S.max_kmh) S.max_kmh = kmh;

    const int z = kmh < ZONE_KMH_1 ? 0 : kmh < ZONE_KMH_2 ? 1 : kmh < ZONE_KMH_3 ? 2 : kmh < ZONE_KMH_4 ? 3 : 4;
    S.zone_m[z] += d;
    if (z == 4) {
      if (!S.in_sprint) { S.in_sprint = true; S.sprint_counted = false; S.sprint_start_ms = now; }
      else if (!S.sprint_counted && now - S.sprint_start_ms >= SPRINT_MIN_MS) { S.sprints++; S.sprint_counted = true; }
    } else {
      S.in_sprint = false;
    }

    // 트랙 점 저장: 움직이는 중 + 마지막 저장점에서 step 이상 벗어났을 때만
    if (v > 0) {
      const bool   first = track.count() == 0;
      const double moved = first ? 1e9 : TinyGPSPlus::distanceBetween(S.last_pt_lat, S.last_pt_lon, S.lat, S.lon);
      if (moved >= S.track_step_m) {
        uint32_t t_s = (now - S.start_ms) / 1000;
        if (t_s > 65535) t_s = 65535;
        float spd_cms = v * 100.0f;
        if (spd_cms > 65535.0f) spd_cms = 65535.0f;
        const bool thinned = track.add((int32_t)lround(S.lat * 1e7), (int32_t)lround(S.lon * 1e7),
                                       (uint16_t)t_s, (uint16_t)spd_cms);
        if (thinned) { S.track_step_m *= 2; Serial.printf("[TRACK] 버퍼 가득 → 절반 솎아냄, step=%.1fm\n", S.track_step_m); }
        S.last_pt_lat = S.lat; S.last_pt_lon = S.lon;
        stored = true;
      }
    }
  }
  pushStatus(stored);
}

// SSE 페이로드 생성 + 전송 (fix 없을 때도 1Hz 하트비트로 호출됨)
static void pushStatus(bool stored) {
  const uint32_t now = millis();
  const uint32_t el  = S.started ? (now - S.start_ms) / 1000 : 0;
  const bool gpsAlive = g_last_gps_byte_ms && (now - g_last_gps_byte_ms) < 3000;
  char utc[16] = "";
  if (gps.time.isValid() && gps.time.age() < 2000)
    snprintf(utc, sizeof utc, "%02u:%02u:%02u.%u", gps.time.hour(), gps.time.minute(), gps.time.second(), gps.time.centisecond() / 10);
  const float alt = (S.fix && gps.altitude.isValid()) ? (float)gps.altitude.meters() : 0.0f;
  snprintf(g_json, sizeof g_json,
    "{\"fix\":%d,\"gps\":%d,\"sat\":%u,\"siv\":%u,\"hdop\":%.1f,\"lat\":%.7f,\"lon\":%.7f,\"alt\":%.1f,\"spd\":%.1f,\"crs\":%.0f,"
    "\"utc\":\"%s\",\"dist\":%.1f,\"max\":%.1f,\"el\":%u,\"z\":[%.0f,%.0f,%.0f,%.0f,%.0f],\"spr\":%u,"
    "\"pl\":%.1f,\"imu\":%d,\"pt\":%d,\"n\":%u,\"nmea\":\"%s\"}",
    S.fix, gpsAlive, (unsigned)gps.satellites.value(), (unsigned)satsInView(), gps.hdop.isValid() ? gps.hdop.hdop() : 99.9,
    S.lat, S.lon, alt, S.fix ? S.kmh : 0.0f, S.course, utc,
    S.dist_m, S.max_kmh, (unsigned)el,
    S.zone_m[0], S.zone_m[1], S.zone_m[2], S.zone_m[3], S.zone_m[4], S.sprints,
    S.player_load, imu.ok(), stored, (unsigned)track.count(), g_nmea_line);   // NMEA는 따옴표·백슬래시가 없어 JSON에 그대로 안전
  if (events.count() > 0) events.send(g_json, "fix", now);
  g_last_push_ms = now;

#if ENABLE_BLE
  // 같은 내용을 BLE 바이너리 프레임으로 (규격: ble_link.h). FIX는 매 호출, STATS는 1Hz
  if (bleConnected()) {
    const float kmh = S.fix ? S.kmh : 0.0f;
    const int z = kmh < ZONE_KMH_1 ? 0 : kmh < ZONE_KMH_2 ? 1 : kmh < ZONE_KMH_3 ? 2 : kmh < ZONE_KMH_4 ? 3 : 4;
    uint8_t f[20];
    f[0] = (S.fix ? 1 : 0) | (gpsAlive ? 2 : 0) | (imu.ok() ? 4 : 0) | (stored ? 8 : 0) | (S.started ? 16 : 0);
    f[1] = clampU8((float)gps.satellites.value());
    f[2] = clampU8((gps.hdop.isValid() ? (float)gps.hdop.hdop() : 25.5f) * 10.0f);
    const uint8_t siv = satsInView();
    f[3] = (uint8_t)(z | ((siv > 31 ? 31 : siv) << 3));   // bit0-2 zone, bit3-7 보이는 위성 수(최대 31)
    putU32(f + 4,  (uint32_t)(int32_t)lround(S.lat * 1e7));
    putU32(f + 8,  (uint32_t)(int32_t)lround(S.lon * 1e7));
    putU16(f + 12, clampU16(kmh / 0.036f));           // km/h → cm/s
    putU16(f + 14, clampU16(S.course * 10.0f));
    putU32(f + 16, (uint32_t)(S.dist_m * 100.0f));    // m → cm
    bleNotifyFix(f, sizeof f);

    static uint32_t last_stats_ms = 0;
    if (now - last_stats_ms >= 1000) {
      last_stats_ms = now;
      uint8_t s[22];
      putU16(s + 0, clampU16(S.max_kmh * 10.0f));
      putU16(s + 2, clampU16((float)el));
      for (int i = 0; i < 5; i++) putU16(s + 4 + i * 2, clampU16(S.zone_m[i]));
      putU16(s + 14, S.sprints);
      putU16(s + 16, clampU16(S.player_load * 10.0f));
      putU16(s + 18, clampU16((float)track.count()));
      putU16(s + 20, (uint16_t)(int16_t)alt);          // 고도 m (부호 있음)
      bleNotifyStats(s, sizeof s);
    }
  }
#endif
}

// IMU 100Hz 샘플링 → PlayerLoad (Catapult 정의: Σ√(Δax²+Δay²+Δaz²)/100, 단위 g)
static void imuTask() {
  static uint32_t next_ms = 0;
  static float    pax = 0, pay = 0, paz = 0;
  static bool     have = false;
  const uint32_t now = millis();
  if (!imu.ok() || (int32_t)(now - next_ms) < 0) return;
  next_ms = now + 1000 / IMU_RATE_HZ;
  float ax, ay, az, gx, gy, gz;
  if (!imu.read(ax, ay, az, gx, gy, gz)) return;
  if (have && S.started) {
    const float dx = ax - pax, dy = ay - pay, dz = az - paz;
    S.player_load += sqrtf(dx * dx + dy * dy + dz * dz) / 100.0f;
  }
  pax = ax; pay = ay; paz = az; have = true;
}

// ============================================================
//  웹 서버
// ============================================================
static void fmtE7(char* out, size_t n, int32_t v) {   // 1e-7도 정수 → "37.1234567"
  const uint32_t a = (uint32_t)(v < 0 ? -(int64_t)v : v);
  snprintf(out, n, "%s%lu.%07lu", v < 0 ? "-" : "", (unsigned long)(a / 10000000UL), (unsigned long)(a % 10000000UL));
}

struct StreamState { size_t i = 0; uint8_t stage = 0; uint32_t gen = 0; size_t n = 0; };

// GET /track → {"pts":[[lat,lon,t_s,spd_cms],...],"n":N}  청크 스트리밍 (수만 점도 RAM 부담 없음)
static void handleTrack(AsyncWebServerRequest* req) {
  auto st = std::make_shared<StreamState>();
  st->gen = track.generation(); st->n = track.count();
  AsyncWebServerResponse* res = req->beginChunkedResponse("application/json",
    [st](uint8_t* buf, size_t maxLen, size_t) -> size_t {
      if (maxLen < 128) return RESPONSE_TRY_AGAIN;
      char* out = (char*)buf; size_t len = 0;
      if (st->stage == 3) return 0;
      if (st->stage == 0) { len += snprintf(out, maxLen, "{\"pts\":["); st->stage = 1; }
      if (st->stage == 1) {
        if (track.generation() != st->gen) st->n = 0;        // 스트리밍 중 리셋/솎아냄 → 여기서 마감
        while (st->i < st->n && st->i < track.count()) {
          if (maxLen - len < 64) return len;                 // 다음 청크에서 계속
          const TrackPt& p = track.at(st->i);
          char la[16], lo[16];
          fmtE7(la, sizeof la, p.lat_e7); fmtE7(lo, sizeof lo, p.lon_e7);
          len += snprintf(out + len, maxLen - len, "%s[%s,%s,%u,%u]", st->i ? "," : "", la, lo, p.t_s, p.spd_cms);
          st->i++;
        }
        st->stage = 2;
      }
      len += snprintf(out + len, maxLen - len, "],\"n\":%u}", (unsigned)st->i);
      st->stage = 3;
      return len;
    });
  res->addHeader("Cache-Control", "no-store");
  req->send(res);
}

// GET /track.gpx → 표준 GPX 1.1 (Google Earth, Strava 등에서 열람)
static void handleGpx(AsyncWebServerRequest* req) {
  auto st = std::make_shared<StreamState>();
  st->gen = track.generation(); st->n = track.count();
  const uint32_t epoch = S.start_epoch;
  AsyncWebServerResponse* res = req->beginChunkedResponse("application/gpx+xml",
    [st, epoch](uint8_t* buf, size_t maxLen, size_t) -> size_t {
      if (maxLen < 256) return RESPONSE_TRY_AGAIN;
      char* out = (char*)buf; size_t len = 0;
      if (st->stage == 3) return 0;
      if (st->stage == 0) {
        len += snprintf(out, maxLen,
          "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
          "<gpx version=\"1.1\" creator=\"FootTrack\" xmlns=\"http://www.topografix.com/GPX/1/1\">\n"
          "<trk><name>FootTrack</name><trkseg>\n");
        st->stage = 1;
      }
      if (st->stage == 1) {
        if (track.generation() != st->gen) st->n = 0;
        while (st->i < st->n && st->i < track.count()) {
          if (maxLen - len < 160) return len;
          const TrackPt& p = track.at(st->i);
          char la[16], lo[16], ts[48] = "";
          fmtE7(la, sizeof la, p.lat_e7); fmtE7(lo, sizeof lo, p.lon_e7);
          if (epoch) {
            time_t t = (time_t)(epoch + p.t_s);
            struct tm tm; gmtime_r(&t, &tm);
            strftime(ts, sizeof ts, "<time>%Y-%m-%dT%H:%M:%SZ</time>", &tm);
          }
          len += snprintf(out + len, maxLen - len, "<trkpt lat=\"%s\" lon=\"%s\">%s</trkpt>\n", la, lo, ts);
          st->i++;
        }
        st->stage = 2;
      }
      len += snprintf(out + len, maxLen - len, "</trkseg></trk></gpx>\n");
      st->stage = 3;
      return len;
    });
  res->addHeader("Content-Disposition", "attachment; filename=\"foottrack.gpx\"");
  req->send(res);
}

static void webSetup() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    AsyncWebServerResponse* res = req->beginResponse(200, "text/html; charset=utf-8",
                                                     (const uint8_t*)INDEX_HTML, strlen_P(INDEX_HTML));
    res->addHeader("Cache-Control", "no-store");
    req->send(res);
  });
  server.on("/track", HTTP_GET, handleTrack);
  server.on("/track.gpx", HTTP_GET, handleGpx);
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest* req) {   // 디버깅용: 마지막 SSE 페이로드
    req->send(200, "application/json", g_json);
  });
  server.on("/reset", HTTP_POST, [](AsyncWebServerRequest* req) {
    resetSession();
    req->send(200, "text/plain", "ok");
  });
  server.onNotFound([](AsyncWebServerRequest* req) { req->send(404, "text/plain", "not found"); });

  events.onConnect([](AsyncEventSourceClient* client) {
    Serial.printf("[SSE] 클라이언트 접속 (%u명)\n", (unsigned)events.count());
    client->send("hello", nullptr, millis(), 1000);
    if (g_json[0]) client->send(g_json, "fix", millis());
  });
  server.addHandler(&events);
  server.begin();
  Serial.println("[WEB] 서버 시작 (/, /events, /track, /track.gpx, /status, POST /reset)");
}

// ============================================================
//  WiFi: 폰 핫스팟(STA) → 실패 시 자체 AP
// ============================================================
static void wifiSetup() {
  WiFi.persistent(false);
  WiFi.setHostname(HOSTNAME);
  if (strlen(WIFI_SSID) > 0) {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("[WiFi] 핫스팟 '%s' 접속 시도", WIFI_SSID);
    const uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < STA_TIMEOUT_MS) { delay(250); Serial.print('.'); }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      g_wifi_sta = true;
      const String ip = WiFi.localIP().toString();
      Serial.printf("[WiFi] 연결됨  →  http://%s/  (또는 http://%s.local/)\n", ip.c_str(), HOSTNAME);
    } else {
      Serial.println("[WiFi] 핫스팟 접속 실패 → AP 모드로 전환");
    }
  }
  if (!g_wifi_sta) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("[WiFi] AP 모드  SSID=%s  PW=%s  →  http://%s/\n", AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str());
  }
#if ENABLE_BLE
  WiFi.setSleep(true);                          // BLE 공존 시 모뎀 슬립 필수 (끄면 IDF가 abort) — SSE 지연 수십 ms 추가
#else
  WiFi.setSleep(false);                         // 실시간 SSE 지연 최소화 (전류 +수십 mA)
#endif
  if (MDNS.begin(HOSTNAME)) MDNS.addService("http", "tcp", 80);
}

// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[FootTrack] ESP32-S3 + ATGM336H + ICM-42688-P");
  Serial.printf("[MEM] PSRAM %s, free heap %u\n", psramFound() ? "있음" : "없음", (unsigned)ESP.getFreeHeap());

  if (!track.begin(TRACK_CAP_SRAM, TRACK_CAP_PSRAM)) Serial.println("[TRACK] 버퍼 할당 실패!");
  else Serial.printf("[TRACK] %u점 버퍼 (%s)\n", (unsigned)track.capacity(), track.inPsram() ? "PSRAM" : "SRAM");

  g_gps_found = gpsProbe();

  if (imu.begin(PIN_IMU_SCK, PIN_IMU_MISO, PIN_IMU_MOSI, PIN_IMU_CS)) Serial.println("[IMU] ICM-42688-P OK (±16g/±2000dps, 100Hz)");
  else Serial.println("[IMU] 감지 실패 — GPS만으로 동작 (PlayerLoad 없음)");

#if ENABLE_BLE
  bleSetup(BLE_NAME, &track, resetSession);
#endif
#if ENABLE_WIFI
  wifiSetup();
  webSetup();
#else
  WiFi.mode(WIFI_OFF);
  Serial.println("[WiFi] 비활성 (ENABLE_WIFI=0, BLE 전용)");
#endif
  pushStatus(false);
}

void loop() {
  // 1) GPS 바이트 소비
  while (GpsSerial.available()) {
    g_last_gps_byte_ms = millis();
    const char c = (char)GpsSerial.read();
    gps.encode(c);
    if (c == '$') g_nmea_len = 0;
    if (c == '\n') {                                   // 문장 완성 → 진단용 보관
      g_nmea_buf[g_nmea_len] = 0; memcpy(g_nmea_line, g_nmea_buf, sizeof g_nmea_line); g_nmea_count++; g_nmea_len = 0;
    } else if (c >= 32 && g_nmea_len < sizeof g_nmea_buf - 1) {
      g_nmea_buf[g_nmea_len++] = c;
    }
  }
  // 2) fix 있는 RMC마다 틱 (TinyGPSPlus는 fix 없으면 speed를 commit하지 않음)
  if (gps.speed.isUpdated()) onGpsTick();
  // 3) IMU 100Hz
  imuTask();
#if ENABLE_BLE
  bleLoop();                                         // 리셋 명령 처리, 이력 재생 진행
#endif
  // 4) fix 없어도 1Hz 하트비트 (위성 수/HDOP/GPS 무응답 표시용)
  if (millis() - g_last_push_ms >= 1000) { S.fix = false; pushStatus(false); }
  // 5) GPS를 못 찾았고 여전히 조용하면 15초마다 재탐색 (배선을 나중에 꽂아도 잡힘)
  static uint32_t last_probe = 0;
  const bool never_seen = g_last_gps_byte_ms == 0;
  const bool stream_lost = g_gps_found && !never_seen && millis() - g_last_gps_byte_ms > 8000;
  if ((!g_gps_found && never_seen && millis() - last_probe >= 15000) || (stream_lost && millis() - last_probe >= 15000)) {
    if (stream_lost) Serial.printf("[GPS] 스트림 끊김 %lus → 재탐색\n", (unsigned long)((millis() - g_last_gps_byte_ms) / 1000));
    last_probe = millis();
    g_gps_found = gpsProbe();
    if (g_gps_found) g_last_gps_byte_ms = millis();
  }
  // 6) 시리얼 로그 5초
  static uint32_t last_log = 0;
  if (millis() - last_log >= 5000) {
    last_log = millis();
    Serial.printf("[LOG] gps=%s fix=%d sat=%u siv=%u hdop=%.1f spd=%.1fkm/h dist=%.0fm pts=%u sse=%u ble=%d/mtu%u heap=%u chars=%lu bad=%lu nmea=%lu\n",
                  (millis() - g_last_gps_byte_ms) < 3000 ? "ok" : "NONE", S.fix,
                  (unsigned)gps.satellites.value(), (unsigned)satsInView(), gps.hdop.isValid() ? gps.hdop.hdop() : 99.9, S.kmh, S.dist_m,
                  (unsigned)track.count(), (unsigned)events.count(),
#if ENABLE_BLE
                  bleConnected(), (unsigned)bleMtu(),
#else
                  0, 0u,
#endif
                  (unsigned)ESP.getFreeHeap(),
                  (unsigned long)gps.charsProcessed(), (unsigned long)gps.failedChecksum(), (unsigned long)g_nmea_count);
    if (g_nmea_line[0]) Serial.printf("      last: %s\n", g_nmea_line);
  }
}
