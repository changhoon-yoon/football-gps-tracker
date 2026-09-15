#include "ble_link.h"
#include <NimBLEDevice.h>

namespace {
const char* UUID_SVC   = "f007ba11-0000-4c45-8000-000000000000";
const char* UUID_FIX   = "f007ba11-0001-4c45-8000-000000000000";
const char* UUID_STATS = "f007ba11-0002-4c45-8000-000000000000";
const char* UUID_HIST  = "f007ba11-0003-4c45-8000-000000000000";
const char* UUID_CTRL  = "f007ba11-0004-4c45-8000-000000000000";

constexpr uint32_t HIST_CHUNK_MS = 30;     // 청크 간격 (연결 간격 7.5~15ms 요청과 맞물려 큐 넘침 방지)
constexpr size_t   HIST_MAX_PTS  = 30;     // 청크당 최대 점 (30×8B = 240B ≤ MTU 247-3)

NimBLECharacteristic* s_fix   = nullptr;
NimBLECharacteristic* s_stats = nullptr;
NimBLECharacteristic* s_hist  = nullptr;
NimBLECharacteristic* s_ctrl  = nullptr;
TrackStore*  s_track = nullptr;
void (*s_onReset)() = nullptr;

volatile bool     s_connected = false;
volatile uint16_t s_mtu = 23;
volatile bool     s_histReq = false;       // NimBLE 태스크 → loop 로 전달되는 요청 플래그
volatile bool     s_resetReq = false;
bool     s_histActive = false;
size_t   s_histIdx = 0, s_histEnd = 0;
uint32_t s_histGen = 0, s_histLastMs = 0;

class ServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* srv, ble_gap_conn_desc* desc) override {
    s_connected = true;
    // 빠른 연결 간격(7.5~15ms) 요청 → 10Hz 알림·이력 재생 지연 최소화
    srv->updateConnParams(desc->conn_handle, 6, 12, 0, 400);
    Serial.println("[BLE] 연결됨");
  }
  void onDisconnect(NimBLEServer* srv) override {
    s_connected = false;
    s_histActive = false;
    Serial.println("[BLE] 끊김 → 광고 재개");
    NimBLEDevice::startAdvertising();
  }
  void onMTUChange(uint16_t mtu, ble_gap_conn_desc* desc) override {
    s_mtu = mtu;
    Serial.printf("[BLE] MTU %u\n", mtu);
  }
};

class CtrlCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c) override {
    NimBLEAttValue v = c->getValue();
    if (v.length() < 1) return;
    const uint8_t cmd = v.data()[0];
    if (cmd == 0x01) s_resetReq = true;
    else if (cmd == 0x02) s_histReq = true;
  }
};

class HistCB : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic* c, ble_gap_conn_desc* desc, uint16_t subValue) override {
    if (subValue & 0x01) s_histReq = true;   // 알림 구독 즉시 이력 재생 시작
  }
};
}  // namespace

void bleSetup(const char* name, TrackStore* track, void (*onReset)()) {
  s_track = track;
  s_onReset = onReset;

  NimBLEDevice::init(name);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setMTU(247);

  NimBLEServer* srv = NimBLEDevice::createServer();
  srv->setCallbacks(new ServerCB());
  NimBLEService* svc = srv->createService(UUID_SVC);
  s_fix   = svc->createCharacteristic(UUID_FIX,   NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  s_stats = svc->createCharacteristic(UUID_STATS, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  s_hist  = svc->createCharacteristic(UUID_HIST,  NIMBLE_PROPERTY::NOTIFY);
  s_ctrl  = svc->createCharacteristic(UUID_CTRL,  NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  s_ctrl->setCallbacks(new CtrlCB());
  s_hist->setCallbacks(new HistCB());
  svc->start();

  // 광고 패킷 = 플래그 + 이름 (폰 선택창에 'FootTrack'이 바로 보이도록), 스캔 응답 = 128bit 서비스 UUID
  // (둘을 합치면 31B를 넘음. Chrome/Android는 두 패킷의 UUID를 합쳐서 필터링하므로 문제 없음)
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  NimBLEAdvertisementData advData;
  advData.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
  advData.setName(name);
  NimBLEAdvertisementData scanData;
  scanData.setCompleteServices(NimBLEUUID(UUID_SVC));
  adv->setAdvertisementData(advData);
  adv->setScanResponseData(scanData);
  adv->start();
  Serial.printf("[BLE] 광고 시작: '%s' (Web Bluetooth 서비스 %s)\n", name, UUID_SVC);
}

void bleNotifyFix(const uint8_t* frame, size_t len) {
  if (!s_connected || !s_fix) return;
  s_fix->setValue(frame, len);
  s_fix->notify();
}

void bleNotifyStats(const uint8_t* frame, size_t len) {
  if (!s_connected || !s_stats) return;
  s_stats->setValue(frame, len);
  s_stats->notify();
}

bool bleConnected() { return s_connected; }
uint16_t bleMtu()   { return s_mtu; }

void bleLoop() {
  if (s_resetReq) { s_resetReq = false; if (s_onReset) s_onReset(); }
  if (s_histReq) {
    s_histReq = false;
    s_histActive = true;
    s_histIdx = 0;
    s_histEnd = s_track ? s_track->count() : 0;
    s_histGen = s_track ? s_track->generation() : 0;
    s_histLastMs = 0;
    Serial.printf("[BLE] 이력 재생 시작: %u점\n", (unsigned)s_histEnd);
  }
  if (!s_histActive) return;
  if (!s_connected) { s_histActive = false; return; }
  if (millis() - s_histLastMs < HIST_CHUNK_MS) return;
  s_histLastMs = millis();

  if (s_track->generation() != s_histGen) s_histEnd = s_histIdx;   // 재생 중 리셋/솎아냄 → 여기서 마감

  uint8_t buf[HIST_MAX_PTS * 8];
  size_t maxPts = (s_mtu > 3 ? (s_mtu - 3) : 20) / 8;
  if (maxPts < 1) maxPts = 1;
  if (maxPts > HIST_MAX_PTS) maxPts = HIST_MAX_PTS;

  size_t n = 0;
  while (s_histIdx < s_histEnd && n < maxPts) {
    const TrackPt& p = s_track->at(s_histIdx++);
    putU32(buf + n * 8, (uint32_t)p.lat_e7);
    putU32(buf + n * 8 + 4, (uint32_t)p.lon_e7);
    n++;
  }
  if (n > 0) { s_hist->setValue(buf, n * 8); s_hist->notify(); }
  if (s_histIdx >= s_histEnd) {
    putU32(buf, 0x7FFFFFFFUL);                 // 끝 마커 (위도로 불가능한 값)
    putU32(buf + 4, (uint32_t)s_histIdx);
    s_hist->setValue(buf, 8);
    s_hist->notify();
    s_histActive = false;
    Serial.printf("[BLE] 이력 재생 완료: %u점\n", (unsigned)s_histIdx);
  }
}
