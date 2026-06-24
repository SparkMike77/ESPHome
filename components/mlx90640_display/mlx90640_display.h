#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/sensor/sensor.h"
#include <Wire.h>
#include <Adafruit_MLX90640.h>

namespace esphome {
namespace mlx90640 {

static const char *const TAG = "mlx90640";

class MLX90640Component : public PollingComponent {
 public:
  MLX90640Component() : PollingComponent(200) {}

  // --- Configuration setters (called from generated code) ---
  void set_mintemp(float t) { mintemp_ = t; }
  void set_maxtemp(float t) { maxtemp_ = t; }
  void set_refresh_rate(uint8_t r) { refresh_rate_ = r; }
  void set_min_temp_sensor(sensor::Sensor *s) { min_temp_sensor_ = s; }
  void set_max_temp_sensor(sensor::Sensor *s) { max_temp_sensor_ = s; }

  // --- Data accessors for display lambda ---
  float get_mintemp() const { return mintemp_; }
  float get_maxtemp() const { return maxtemp_; }
  float get_min_detected() const { return min_detected_; }
  float get_max_detected() const { return max_detected_; }
  const float *get_frame() const { return frame_; }
  bool is_ready() const { return ready_; }

  void setup() override {
    Wire.begin(21, 22);
    Wire.setClock(400000);
    if (!mlx_.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
      ESP_LOGE(TAG, "MLX90640 not found at 0x33 — check wiring");
      this->mark_failed();
      return;
    }
    mlx_.setMode(MLX90640_CHESS);
    mlx_.setResolution(MLX90640_ADC_18BIT);
    mlx_.setRefreshRate(static_cast<mlx90640_refreshrate_t>(refresh_rate_));
    ESP_LOGI(TAG, "MLX90640 ready, refresh_rate register=0x%02X", refresh_rate_);
  }

  void update() override {
    float tmp[768];
    if (mlx_.getFrame(tmp) != 0) {
      ESP_LOGW(TAG, "Frame read failed — skipping");
      return;
    }
    memcpy(frame_, tmp, sizeof(frame_));

    float lo = 300.0f, hi = -100.0f;
    for (int i = 0; i < 768; i++) {
      if (frame_[i] < lo) lo = frame_[i];
      if (frame_[i] > hi) hi = frame_[i];
    }
    min_detected_ = lo;
    max_detected_ = hi;

    if (min_temp_sensor_ != nullptr) min_temp_sensor_->publish_state(lo);
    if (max_temp_sensor_ != nullptr) max_temp_sensor_->publish_state(hi);
    ready_ = true;
  }

  // Iron palette: black(0) → blue(0.25) → cyan(0.5) → yellow(0.75) → red(1.0)
  static Color temp_to_color(float temp, float t_min, float t_max) {
    float norm = (temp - t_min) / (t_max - t_min);
    if (norm < 0.0f) norm = 0.0f;
    if (norm > 1.0f) norm = 1.0f;

    static const float stops[5] = {0.00f, 0.25f, 0.50f, 0.75f, 1.00f};
    static const uint8_t rr[5]  = {   0,    0,    0,  255,  255};
    static const uint8_t gg[5]  = {   0,    0,  255,  255,    0};
    static const uint8_t bb[5]  = {   0,  255,  255,    0,    0};

    int seg = 0;
    for (int i = 1; i < 4; i++) {
      if (norm >= stops[i]) seg = i;
    }
    float t = (norm - stops[seg]) / (stops[seg + 1] - stops[seg]);

    return Color(
        static_cast<uint8_t>(rr[seg] + t * static_cast<float>(static_cast<int>(rr[seg + 1]) - rr[seg])),
        static_cast<uint8_t>(gg[seg] + t * static_cast<float>(static_cast<int>(gg[seg + 1]) - gg[seg])),
        static_cast<uint8_t>(bb[seg] + t * static_cast<float>(static_cast<int>(bb[seg + 1]) - bb[seg]))
    );
  }

 protected:
  Adafruit_MLX90640 mlx_;
  float frame_[768]{};
  float mintemp_{15.0f};
  float maxtemp_{40.0f};
  uint8_t refresh_rate_{0x04};
  float min_detected_{0.0f};
  float max_detected_{100.0f};
  bool ready_{false};
  sensor::Sensor *min_temp_sensor_{nullptr};
  sensor::Sensor *max_temp_sensor_{nullptr};
};

}  // namespace mlx90640
}  // namespace esphome
