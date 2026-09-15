// ============================================================
//  TrackStore — 동선 점 버퍼 (PSRAM 우선, 없으면 SRAM)
//  가득 차면 짝수 인덱스만 남겨 절반으로 솎아낸다(해상도 절반, 기록 유지).
// ============================================================
#pragma once
#include <Arduino.h>

struct TrackPt {
  int32_t  lat_e7;   // 위도 × 1e7 (float 정밀도 부족 → 정수 고정소수)
  int32_t  lon_e7;   // 경도 × 1e7
  uint16_t t_s;      // 세션 시작 후 초 (최대 18시간)
  uint16_t spd_cms;  // 속도 cm/s
};

class TrackStore {
 public:
  bool begin(size_t capSram, size_t capPsram) {
    if (psramFound()) {
      pts_ = (TrackPt*)ps_malloc(sizeof(TrackPt) * capPsram);
      if (pts_) { cap_ = capPsram; psram_ = true; }
    }
    if (!pts_) {
      pts_ = (TrackPt*)malloc(sizeof(TrackPt) * capSram);
      if (pts_) cap_ = capSram;
    }
    n_ = 0; gen_ = 0; thinned_ = 0;
    return pts_ != nullptr;
  }

  void clear() { n_ = 0; gen_++; thinned_ = 0; }

  // 반환값: 이번 add에서 솎아내기가 일어났으면 true
  bool add(int32_t lat, int32_t lon, uint16_t t, uint16_t spd) {
    if (!pts_) return false;
    bool thinned = false;
    if (n_ >= cap_) {
      size_t j = 0;
      for (size_t i = 0; i < n_; i += 2) pts_[j++] = pts_[i];
      n_ = j; gen_++; thinned_++; thinned = true;
    }
    pts_[n_] = {lat, lon, t, spd};
    n_ = n_ + 1;
    return thinned;
  }

  size_t   count() const      { return n_; }
  size_t   capacity() const   { return cap_; }
  uint32_t generation() const { return gen_; }   // clear/솎아내기마다 증가 — 스트리밍 중 변경 감지용
  uint8_t  thinned() const    { return thinned_; }
  bool     inPsram() const    { return psram_; }
  const TrackPt& at(size_t i) const { return pts_[i]; }

 private:
  TrackPt* pts_ = nullptr;
  size_t   cap_ = 0;
  volatile size_t   n_ = 0;
  volatile uint32_t gen_ = 0;
  uint8_t  thinned_ = 0;
  bool     psram_ = false;
};
