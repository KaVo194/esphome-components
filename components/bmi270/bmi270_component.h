#pragma once
#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"

extern "C" {
  #include "bmi2.h"
  #include "bmi270.h"
// Forward decls (harmless if already present)
  int8_t bmi270_init(struct bmi2_dev *dev);
  int8_t bmi270_load_config_file(struct bmi2_dev *dev);
}

namespace esphome {
namespace bmi270 {

class BMI270Component : public PollingComponent, public i2c::I2CDevice {
 public:
  // published sensors
  sensor::Sensor *accel_x{nullptr}, *accel_y{nullptr}, *accel_z{nullptr};
  sensor::Sensor *gyro_x{nullptr},  *gyro_y{nullptr},  *gyro_z{nullptr};

  void set_odr(uint16_t hz) { odr_hz_ = hz; }
  // called from sensor.py (already added earlier)
  void set_accel_x(sensor::Sensor *s) { accel_x = s; }
  void set_accel_y(sensor::Sensor *s) { accel_y = s; }
  void set_accel_z(sensor::Sensor *s) { accel_z = s; }
  void set_gyro_x(sensor::Sensor *s)  { gyro_x  = s; }
  void set_gyro_y(sensor::Sensor *s)  { gyro_y  = s; }
  void set_gyro_z(sensor::Sensor *s)  { gyro_z  = s; }

  void setup() override;
  void update() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

 private:
  // Bosch context
  bmi2_dev dev_{};
  uint16_t odr_hz_{100};

  // Bosch callbacks (static trampolines)
  static int8_t cb_read(uint8_t reg, uint8_t *data, uint32_t len, void *intf_ptr);
  static int8_t cb_write(uint8_t reg, const uint8_t *data, uint32_t len, void *intf_ptr);
  static void   cb_delay_us(uint32_t period, void *intf_ptr);

  // instance helpers used by callbacks
  int8_t read_regs_(uint8_t reg, uint8_t *data, uint32_t len);
  int8_t write_regs_(uint8_t reg, const uint8_t *data, uint32_t len);

  // small helpers
  uint8_t map_acc_odr_(uint16_t hz) const;
  uint8_t map_gyr_odr_(uint16_t hz) const;
};

}  // namespace bmi270
}  // namespace esphome
