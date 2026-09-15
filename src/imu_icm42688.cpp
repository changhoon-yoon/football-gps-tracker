#include "imu_icm42688.h"

namespace {
constexpr uint8_t REG_DEVICE_CONFIG = 0x11;   // bit0: soft reset
constexpr uint8_t REG_ACCEL_DATA_X1 = 0x1F;   // 0x1F..0x2A: AX AY AZ GX GY GZ (int16 big-endian)
constexpr uint8_t REG_PWR_MGMT0     = 0x4E;
constexpr uint8_t REG_GYRO_CONFIG0  = 0x4F;
constexpr uint8_t REG_ACCEL_CONFIG0 = 0x50;
constexpr uint8_t REG_WHO_AM_I      = 0x75;
constexpr uint8_t REG_BANK_SEL      = 0x76;
constexpr uint8_t WHO_AM_I_VAL      = 0x47;

constexpr uint8_t ODR_100HZ   = 0x08;         // [3:0] ODR 코드
constexpr uint8_t GYRO_FS_2000 = 0x0 << 5;    // [7:5] FS_SEL
constexpr uint8_t ACCEL_FS_16G = 0x0 << 5;

constexpr float ACCEL_LSB_PER_G   = 2048.0f;  // ±16g
constexpr float GYRO_LSB_PER_DPS  = 16.4f;    // ±2000dps
constexpr uint32_t SPI_HZ = 8000000;          // 최대 24MHz, 배선 여유로 8MHz
}  // namespace

bool ICM42688::begin(int sck, int miso, int mosi, int cs) {
  cs_ = cs;
  pinMode(cs_, OUTPUT);
  digitalWrite(cs_, HIGH);
  spi_.begin(sck, miso, mosi, cs_);
  delay(5);

  writeReg(REG_BANK_SEL, 0x00);
  writeReg(REG_DEVICE_CONFIG, 0x01);          // soft reset (1ms 대기 규정)
  delay(5);

  const uint8_t id = readReg(REG_WHO_AM_I);
  if (id != WHO_AM_I_VAL) {
    log_e("ICM-42688-P WHO_AM_I=0x%02X (expected 0x47)", id);
    ok_ = false;
    return false;
  }

  writeReg(REG_BANK_SEL, 0x00);
  writeReg(REG_GYRO_CONFIG0,  GYRO_FS_2000 | ODR_100HZ);
  writeReg(REG_ACCEL_CONFIG0, ACCEL_FS_16G | ODR_100HZ);
  writeReg(REG_PWR_MGMT0, 0x0F);              // gyro LN(11) + accel LN(11)
  delay(100);                                 // 자이로 기동 45ms + 여유
  ok_ = true;
  return true;
}

bool ICM42688::read(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
  if (!ok_) return false;
  uint8_t b[12];
  readRegs(REG_ACCEL_DATA_X1, b, sizeof b);
  auto s16 = [&](int i) { return (int16_t)(((uint16_t)b[i] << 8) | b[i + 1]); };
  const int16_t rax = s16(0);
  if (rax == -32768) return false;            // 0x8000 = 데이터 무효 마커
  ax = rax     / ACCEL_LSB_PER_G;
  ay = s16(2)  / ACCEL_LSB_PER_G;
  az = s16(4)  / ACCEL_LSB_PER_G;
  gx = s16(6)  / GYRO_LSB_PER_DPS;
  gy = s16(8)  / GYRO_LSB_PER_DPS;
  gz = s16(10) / GYRO_LSB_PER_DPS;
  return true;
}

uint8_t ICM42688::readReg(uint8_t reg) {
  uint8_t v = 0;
  readRegs(reg, &v, 1);
  return v;
}

void ICM42688::readRegs(uint8_t reg, uint8_t* buf, size_t n) {
  spi_.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(cs_, LOW);
  spi_.transfer(reg | 0x80);                  // MSB=1 → read
  for (size_t i = 0; i < n; i++) buf[i] = spi_.transfer(0x00);
  digitalWrite(cs_, HIGH);
  spi_.endTransaction();
}

void ICM42688::writeReg(uint8_t reg, uint8_t val) {
  spi_.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(cs_, LOW);
  spi_.transfer(reg & 0x7F);
  spi_.transfer(val);
  digitalWrite(cs_, HIGH);
  spi_.endTransaction();
  delayMicroseconds(300);                     // PWR_MGMT0 변경 후 200µs 쓰기 금지 규정
}
