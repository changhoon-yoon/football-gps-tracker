// ============================================================
//  ICM-42688-P 최소 드라이버 (SPI, 뱅크0 레지스터 직접 제어)
//  accel ±16g / gyro ±2000dps / 100Hz Low-Noise. FIFO·APEX는 미사용.
// ============================================================
#pragma once
#include <Arduino.h>
#include <SPI.h>

class ICM42688 {
 public:
  explicit ICM42688(SPIClass& spi = SPI) : spi_(spi) {}
  bool begin(int sck, int miso, int mosi, int cs);                 // WHO_AM_I(0x47) 확인 실패 시 false
  bool read(float& ax, float& ay, float& az, float& gx, float& gy, float& gz);  // 단위: g, dps
  bool ok() const { return ok_; }

 private:
  uint8_t readReg(uint8_t reg);
  void    readRegs(uint8_t reg, uint8_t* buf, size_t n);
  void    writeReg(uint8_t reg, uint8_t val);

  SPIClass& spi_;
  int  cs_ = -1;
  bool ok_ = false;
};
