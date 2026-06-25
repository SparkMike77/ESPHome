#pragma once

#include "esphome/core/component.h"
#include "esphome/core/color.h"
#include "esphome/components/sensor/sensor.h"
#include "mlx90640_api.h"

namespace esphome {
namespace mlx90640 {

class MLX90640Component : public PollingComponent {
 public:
  MLX90640Component() : PollingComponent(200) {}

  void set_mintemp(float t) { mintemp_ = t; }
  void set_maxtemp(float t) { maxtemp_ = t; }
  void set_refresh_rate(uint8_t r) { refresh_rate_ = r; }
  void set_min_temp_sensor(sensor::Sensor *s) { min_temp_sensor_ = s; }
  void set_max_temp_sensor(sensor::Sensor *s) { max_temp_sensor_ = s; }

  float get_mintemp() const { return mintemp_; }
  float get_maxtemp() const { return maxtemp_; }
  float get_min_detected() const { return min_detected_; }
  float get_max_detected() const { return max_detected_; }
  const float *get_frame() const { return frame_; }
  bool is_ready() const { return ready_; }

  void setup() override;
  void update() override;

  static Color temp_to_color(float temp, float t_min, float t_max);

 protected:
  paramsMLX90640 params_{};
  uint16_t       eeData_[MLX90640_EEPROM_WORDS]{};
  uint16_t       frameData_[MLX90640_FRAME_WORDS]{};
  float          frame_[768]{};

  float    mintemp_{15.0f};
  float    maxtemp_{40.0f};
  uint8_t  refresh_rate_{0x04};
  float    min_detected_{0.0f};
  float    max_detected_{100.0f};
  bool     ready_{false};

  sensor::Sensor *min_temp_sensor_{nullptr};
  sensor::Sensor *max_temp_sensor_{nullptr};
};

}  // namespace mlx90640
}  // namespace esphome
