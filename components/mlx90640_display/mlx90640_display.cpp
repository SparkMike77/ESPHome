#include "mlx90640_display.h"
#include "mlx90640_i2c.h"
#include "mlx90640_api.h"
#include "esphome/core/log.h"

namespace esphome {
namespace mlx90640 {

static const char *const TAG = "mlx90640";
static const uint8_t I2C_ADDR = MLX90640_I2CADDR_DEFAULT;

void MLX90640Component::setup() {
    MLX90640_I2CInit(21, 22, 400000);

    if (MLX90640_DumpEE(I2C_ADDR, eeData_) != 0) {
        ESP_LOGE(TAG, "EEPROM read failed — check wiring");
        this->mark_failed();
        return;
    }
    if (MLX90640_ExtractParameters(eeData_, &params_) != 0) {
        ESP_LOGE(TAG, "Parameter extraction failed");
        this->mark_failed();
        return;
    }
    MLX90640_SetChessMode(I2C_ADDR);
    MLX90640_SetRefreshRate(I2C_ADDR, refresh_rate_);
    ESP_LOGI(TAG, "MLX90640 ready (refresh_rate=0x%02X)", refresh_rate_);
}

void MLX90640Component::update() {
    // Collect two sub-pages so every pixel is updated once per call.
    for (int sp = 0; sp < 2; sp++) {
        int ret = MLX90640_GetFrameData(I2C_ADDR, frameData_);
        if (ret < 0) {
            ESP_LOGW(TAG, "Frame read error %d — skipping", ret);
            return;
        }
        float tr = MLX90640_GetTa(frameData_, &params_) - 8.0f;
        MLX90640_CalculateTo(frameData_, &params_, 0.95f, tr, frame_);
    }

    float lo = 300.0f, hi = -100.0f;
    for (int i = 0; i < 768; i++) {
        if (frame_[i] < lo) lo = frame_[i];
        if (frame_[i] > hi) hi = frame_[i];
    }
    min_detected_ = lo;
    max_detected_ = hi;

    if (min_temp_sensor_) min_temp_sensor_->publish_state(lo);
    if (max_temp_sensor_) max_temp_sensor_->publish_state(hi);
    ready_ = true;
}

Color MLX90640Component::temp_to_color(float temp, float t_min, float t_max) {
    float norm = (temp - t_min) / (t_max - t_min);
    if (norm < 0.0f) norm = 0.0f;
    if (norm > 1.0f) norm = 1.0f;

    static const float stops[5] = {0.00f, 0.25f, 0.50f, 0.75f, 1.00f};
    static const uint8_t rr[5]  = {  0,   0,   0, 255, 255};
    static const uint8_t gg[5]  = {  0,   0, 255, 255,   0};
    static const uint8_t bb[5]  = {  0, 255, 255,   0,   0};

    int seg = 0;
    for (int i = 1; i < 4; i++) {
        if (norm >= stops[i]) seg = i;
    }
    float t = (norm - stops[seg]) / (stops[seg + 1] - stops[seg]);
    return Color(
        (uint8_t)(rr[seg] + t * (int)(rr[seg + 1] - rr[seg])),
        (uint8_t)(gg[seg] + t * (int)(gg[seg + 1] - gg[seg])),
        (uint8_t)(bb[seg] + t * (int)(bb[seg + 1] - bb[seg]))
    );
}

}  // namespace mlx90640
}  // namespace esphome
