#include "bmi270.h"
#include "esphome/core/log.h"
#include <cstring>
#include <cmath>

namespace esphome {
namespace bmi270 {

static const char *const TAG = "bmi270";

// ---------- Bosch callback trampolines ----------
int8_t BMI270Component::cb_read(uint8_t reg, uint8_t *data, uint32_t len, void *intf_ptr) {
  auto *self = static_cast<BMI270Component *>(intf_ptr);
  return self->read_regs_(reg, data, len);
}
int8_t BMI270Component::cb_write(uint8_t reg, const uint8_t *data, uint32_t len, void *intf_ptr) {
  auto *self = static_cast<BMI270Component *>(intf_ptr);
  return self->write_regs_(reg, data, len);
}
void BMI270Component::cb_delay_us(uint32_t us, void * /*intf_ptr*/) {
  // ESPHome delay is ms; round up
  delay((us + 999) / 1000);
}

// ---------- Low-level I2C helpers ----------
int8_t BMI270Component::read_regs_(uint8_t reg, uint8_t *data, uint32_t len) {
  if (!this->write(&reg, 1)) return BMI2_E_COM_FAIL;
  if (!this->read(data, len)) return BMI2_E_COM_FAIL;
  return BMI2_OK;
}
int8_t BMI270Component::write_regs_(uint8_t reg, const uint8_t *data, uint32_t len) {
  // write [reg | payload...]
  uint8_t buf[1 + 64];
  if (len > 64) return BMI2_E_COM_FAIL;
  buf[0] = reg;
  memcpy(&buf[1], data, len);
  if (!this->write(buf, 1 + len)) return BMI2_E_COM_FAIL;
  return BMI2_OK;
}

// ---------- ODR maps (simple nearest selection) ----------
uint8_t BMI270Component::map_acc_odr_(uint16_t hz) const {
  if (hz >= 1600) return BMI2_ACC_ODR_1600HZ;
  if (hz >= 800)  return BMI2_ACC_ODR_800HZ;
  if (hz >= 400)  return BMI2_ACC_ODR_400HZ;
  if (hz >= 200)  return BMI2_ACC_ODR_200HZ;
  if (hz >= 100)  return BMI2_ACC_ODR_100HZ;
  if (hz >= 50)   return BMI2_ACC_ODR_50HZ;
  if (hz >= 25)   return BMI2_ACC_ODR_25HZ;
  return BMI2_ACC_ODR_12_5HZ;
}
uint8_t BMI270Component::map_gyr_odr_(uint16_t hz) const {
  if (hz >= 1600) return BMI2_GYR_ODR_1600HZ;
  if (hz >= 800)  return BMI2_GYR_ODR_800HZ;
  if (hz >= 400)  return BMI2_GYR_ODR_400HZ;
  if (hz >= 200)  return BMI2_GYR_ODR_200HZ;
  if (hz >= 100)  return BMI2_GYR_ODR_100HZ;
  if (hz >= 50)   return BMI2_GYR_ODR_50HZ;
  if (hz >= 25)   return BMI2_GYR_ODR_25HZ;
  return BMI2_GYR_ODR_25HZ; // gyro has no 12.5, use 25 as floor
}

// ---------- Setup ----------
void BMI270Component::setup() {
  ESP_LOGI(TAG, "Setting up BMI270...");

  // Fill Bosch device struct
  dev_.intf             = BMI2_I2C_INTF;
  dev_.intf_ptr         = this;                 // back-pointer for callbacks
  dev_.read             = &BMI270Component::cb_read;
  dev_.write            = &BMI270Component::cb_write;
  dev_.delay_us         = &BMI270Component::cb_delay_us;
  dev_.read_write_len   = 64;                   // safe for our buffers

  // (Optional sanity) read chip id first
  uint8_t id_reg = BMI2_CHIP_ID_ADDR; // 0x00
  uint8_t chip_id = 0;
  if (!this->write(&id_reg, 1) || !this->read(&chip_id, 1)) {
    ESP_LOGE(TAG, "I2C chip-id read failed");
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "Chip ID: 0x%02X (expected 0x24)", chip_id);

  // Init core + load config (required)
  int8_t rslt = bmi270_init(&dev_);
  if (rslt != BMI2_OK) { ESP_LOGE(TAG, "bmi270_init failed: %d", rslt); this->mark_failed(); return; }

  rslt = bmi270_load_config_file(&dev_);
  if (rslt != BMI2_OK) { ESP_LOGE(TAG, "config load failed: %d", rslt); this->mark_failed(); return; }

  // Enable accel + gyro
  uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };
  rslt = bmi2_sensor_enable(sens_list, 2, &dev_);
  if (rslt != BMI2_OK) { ESP_LOGE(TAG, "sensor_enable failed: %d", rslt); this->mark_failed(); return; }

  // Get current configs, modify, and set back
  bmi2_sens_config cfg[2]{};
  cfg[0].type = BMI2_ACCEL; cfg[1].type = BMI2_GYRO;
  rslt = bmi2_get_sensor_config(cfg, 2, &dev_);
  if (rslt != BMI2_OK) { ESP_LOGE(TAG, "get_config failed: %d", rslt); this->mark_failed(); return; }

  // Accel: ±2g, chosen bandwidth, ODR
  cfg[0].cfg.acc.range = BMI2_ACC_RANGE_2G;
  cfg[0].cfg.acc.bwp   = BMI2_ACC_OSR4_AVG1;               // bandwidth/power mode
  cfg[0].cfg.acc.odr   = map_acc_odr_(odr_hz_);

  // Gyro: ±2000 dps, bandwidth, ODR
  cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
  cfg[1].cfg.gyr.bwp   = BMI2_GYR_OSR4_MODE;
  cfg[1].cfg.gyr.odr   = map_gyr_odr_(odr_hz_);

  rslt = bmi2_set_sensor_config(cfg, 2, &dev_);
  if (rslt != BMI2_OK) { ESP_LOGE(TAG, "set_config failed: %d", rslt); this->mark_failed(); return; }

  ESP_LOGI(TAG, "BMI270 setup OK (ODR=%u Hz, ranges: ±2g / ±2000 dps)", odr_hz_);
}

// ---------- Update ----------
void BMI270Component::update() {
  bmi2_sens_data data{};
  data.type = BMI2_ACCEL | BMI2_GYRO;

  int8_t rslt = bmi2_get_sensor_data(&data, &dev_);
  if (rslt != BMI2_OK) {
    ESP_LOGW(TAG, "get_sensor_data failed: %d", rslt);
    return;
  }

  // Scale raw -> SI units
  // NOTE: Confirm LSB/scale from Bosch docs for your chosen ranges.
  // Common values (same as BMI160):
  //  accel ±2g  => 16384 LSB/g
  //  gyro  ±2000 dps => 16.4 LSB/(°/s)
  auto to_ms2 = [](int16_t raw)->float { return (raw / 16384.0f) * 9.80665f; };
  auto to_dps = [](int16_t raw)->float { return (raw / 16.4f); };

  if (accel_x) accel_x->publish_state(to_ms2(data.acc.x));
  if (accel_y) accel_y->publish_state(to_ms2(data.acc.y));
  if (accel_z) accel_z->publish_state(to_ms2(data.acc.z));
  if (gyro_x)  gyro_x->publish_state(to_dps(data.gyr.x));
  if (gyro_y)  gyro_y->publish_state(to_dps(data.gyr.y));
  if (gyro_z)  gyro_z->publish_state(to_dps(data.gyr.z));
}

}  // namespace bmi270
}  // namespace esphome
