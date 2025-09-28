#include"esphome/core/hal.h"
#include "esphome/core/log.h"
#include "bmi270_component.h"
#include <cstring>
#include <cmath>

namespace esphome {
namespace bmi270 {

static const char *const TAG = "bmi270";

static const char* bmi2_errstr(int8_t r) {
  if (r == BMI2_OK) return "OK";

  #ifdef BMI2_E_NULL_PTR
  if (r == BMI2_E_NULL_PTR) return "NULL_PTR";
  #endif

  #ifdef BMI2_E_COM_FAIL
  if (r == BMI2_E_COM_FAIL) return "COM_FAIL";
  #endif

  #ifdef BMI2_E_DEV_NOT_FOUND
  if (r == BMI2_E_DEV_NOT_FOUND) return "DEV_NOT_FOUND";
  #endif

  #ifdef BMI2_E_INVALID_SENSOR
  if (r == BMI2_E_INVALID_SENSOR) return "INVALID_SENSOR";
  #endif

  #ifdef BMI2_E_CONFIG_LOAD
  if (r == BMI2_E_CONFIG_LOAD) return "CONFIG_LOAD";
  #endif
  #ifdef BMI2_E_CFG_LOAD
  if (r == BMI2_E_CFG_LOAD) return "CFG_LOAD";
  #endif

  #ifdef BMI2_NOT_INIT
  if (r == BMI2_NOT_INIT) return "NOT_INIT";
  #endif
  #ifdef BMI2_E_INIT
  if (r == BMI2_E_INIT) return "INIT";
  #endif

  #ifdef BMI2_E_INVALID_PAGE
  if (r == BMI2_E_INVALID_PAGE) return "INVALID_PAGE";
  #endif

  #ifdef BMI2_E_INVALID_INPUT
  if (r == BMI2_E_INVALID_INPUT) return "INVALID_INPUT";
  #endif

  #ifdef BMI2_E_INVALID_ODR
  if (r == BMI2_E_INVALID_ODR) return "INVALID_ODR";
  #endif

  #ifdef BMI2_E_INVALID_PARAM
  if (r == BMI2_E_INVALID_PARAM) return "INVALID_PARAM";
  #endif

  #ifdef BMI2_E_SELF_TEST_FAIL
  if (r == BMI2_E_SELF_TEST_FAIL) return "SELF_TEST_FAIL";
  #endif

  return "UNKNOWN";
}

// ---------- Bosch callback trampolines ----------
int8_t BMI270Component::cb_read(uint8_t reg, uint8_t *data, uint32_t len, void *intf_ptr) {
  auto *self = static_cast<BMI270Component *>(intf_ptr);
  return self->read_regs_(reg, data, len);
}
int8_t BMI270Component::cb_write(uint8_t reg, const uint8_t *data, uint32_t len, void *intf_ptr) {
  auto *self = static_cast<BMI270Component *>(intf_ptr);
  return self->write_regs_(reg, data, len);
}
void BMI270Component::cb_delay_us(uint32_t us, void *) {
  delayMicroseconds(us);  // from esphome/core/hal.h
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

void BMI270Component::setup() {
  ESP_LOGI(TAG, "Setting up BMI270 (no external config load)...");

  dev_.intf           = BMI2_I2C_INTF;
  dev_.intf_ptr       = this;
  dev_.read           = &BMI270Component::cb_read;
  dev_.write          = &BMI270Component::cb_write;
  dev_.delay_us       = &BMI270Component::cb_delay_us;
  dev_.read_write_len = 64;

  // Optional: chip id sanity (0x24 expected)
  uint8_t reg = 0x00, id = 0;
  if (!this->write(&reg, 1) || !this->read(&id, 1)) {
    ESP_LOGE(TAG, "I2C chip-id read failed (COM_FAIL)");
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "Chip ID: 0x%02X (expect 0x24)", id);
  if (id != 0x24) {
    ESP_LOGE(TAG, "Unexpected chip id -> wrong address/wiring?");
    this->mark_failed();
    return;
  }

  // (Optional but helps) soft reset then small delay
  int8_t rslt = bmi2_soft_reset(&dev_);
  ESP_LOGI(TAG, "soft_reset -> %d (%s)", rslt, bmi2_errstr(rslt));
  this->cb_delay_us(5000, nullptr);

  // Init
  rslt = bmi270_init(&dev_);
  ESP_LOGI(TAG, "bmi270_init -> %d (%s)", rslt, bmi2_errstr(rslt));
  if (rslt != BMI2_OK) { this->mark_failed(); return; }

  // Enable accel + gyro
  uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };
  rslt = bmi2_sensor_enable(sens_list, 2, &dev_);
  ESP_LOGI(TAG, "sensor_enable(accel,gyro) -> %d (%s)", rslt, bmi2_errstr(rslt));
  if (rslt != BMI2_OK) { this->mark_failed(); return; }

  // Get, tweak, set config
  bmi2_sens_config cfg[2]{};
  cfg[0].type = BMI2_ACCEL; cfg[1].type = BMI2_GYRO;

  rslt = bmi2_get_sensor_config(cfg, 2, &dev_);
  ESP_LOGI(TAG, "get_sensor_config -> %d (%s)", rslt, bmi2_errstr(rslt));
  if (rslt != BMI2_OK) { this->mark_failed(); return; }

  cfg[0].cfg.acc.range = BMI2_ACC_RANGE_2G;
  cfg[0].cfg.acc.bwp   = BMI2_ACC_OSR4_AVG1;
  cfg[0].cfg.acc.odr   = map_acc_odr_(odr_hz_);

  cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
  cfg[1].cfg.gyr.bwp   = BMI2_GYR_OSR4_MODE;
  cfg[1].cfg.gyr.odr   = map_gyr_odr_(odr_hz_);

  rslt = bmi2_set_sensor_config(cfg, 2, &dev_);
  ESP_LOGI(TAG, "set_sensor_config -> %d (%s)", rslt, bmi2_errstr(rslt));
  if (rslt != BMI2_OK) { this->mark_failed(); return; }

  ESP_LOGI(TAG, "BMI270 setup complete.");
}

// ---------- Update ----------
void BMI270Component::update() {
  bmi2_sens_data data{};
  int8_t rslt = bmi2_get_sensor_data(&data, &dev_);
  if (rslt != BMI2_OK) {
    ESP_LOGW(TAG, "bmi2_get_sensor_data failed: %d", rslt);
    return;
  }

  auto to_ms2 = [](int16_t r){ return (r / 16384.0f) * 9.80665f; }; // ±2g
  auto to_dps = [](int16_t r){ return (r / 16.4f); };               // ±2000 dps

  if (accel_x) accel_x->publish_state(to_ms2(data.acc.x));
  if (accel_y) accel_y->publish_state(to_ms2(data.acc.y));
  if (accel_z) accel_z->publish_state(to_ms2(data.acc.z));
  if (gyro_x)  gyro_x->publish_state(to_dps(data.gyr.x));
  if (gyro_y)  gyro_y->publish_state(to_dps(data.gyr.y));
  if (gyro_z)  gyro_z->publish_state(to_dps(data.gyr.z));
}
}  // namespace bmi270
}  // namespace esphome
