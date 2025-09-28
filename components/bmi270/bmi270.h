#pragma once
#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace bmi270 {

class BMI270Component : public PollingComponent, public i2c::I2CDevice {
 public:
  void set_accel_x(sensor::Sensor *s) { accel_x = s; }
  void set_accel_y(sensor::Sensor *s) { accel_y = s; }
  void set_accel_z(sensor::Sensor *s) { accel_z = s; }
  void set_gyro_x(sensor::Sensor *s)  { gyro_x  = s; }
  void set_gyro_y(sensor::Sensor *s)  { gyro_y  = s; }
  void set_gyro_z(sensor::Sensor *s)  { gyro_z  = s; }

  sensor::Sensor *accel_x{nullptr}, *accel_y{nullptr}, *accel_z{nullptr};
  sensor::Sensor *gyro_x{nullptr},  *gyro_y{nullptr},  *gyro_z{nullptr};

  void set_odr(uint16_t odr) { odr_hz_ = odr; }

  void setup() override;
  void update() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

 private:
  uint16_t odr_hz_{100};
};

}  // namespace bmi270
}  // namespace esphome
