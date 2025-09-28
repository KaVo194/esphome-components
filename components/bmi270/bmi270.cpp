#include "bmi270.h"
#include "esphome/core/log.h"

namespace esphome {
namespace bmi270 {

static const char *const TAG = "bmi270";

void BMI270Component::setup() {
  ESP_LOGI(TAG, "BMI270 stub setup (ODR=%u Hz) — wiring & build path OK.", odr_hz_);
  // In a real driver you would ping I2C here; for a stub we just succeed.
}

void BMI270Component::update() {
  // Publish zeros so you see sensors arriving; replace with real reads later.
  if (accel_x) accel_x->publish_state(0.0f);
  if (accel_y) accel_y->publish_state(0.0f);
  if (accel_z) accel_z->publish_state(9.80665f);  // pretend flat (1g on Z)
  if (gyro_x)  gyro_x->publish_state(0.0f);
  if (gyro_y)  gyro_y->publish_state(0.0f);
  if (gyro_z)  gyro_z->publish_state(0.0f);
}

}  // namespace bmi270
}  // namespace esphome
