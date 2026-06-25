/*
 * ESPectre - Calibration Buffer Implementation
 *
 * Original (GPLv3) by Francesco Pace stored calibration data on SPIFFS. CrowPanel
 * prop variant: RAM-backed. The C6 is updated via esp-hosted slave-OTA, which only
 * rewrites the app partition (it can't add a SPIFFS partition), so the buffer lives
 * in heap for the brief calibration and is freed afterwards.
 */
#include "calibration_file_buffer.h"
#include "utils.h"
#include "esphome/core/log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>
#include <cmath>
#include <algorithm>

namespace esphome {
namespace espectre {

static const char *TAG = "CalRamBuffer";

#define PKT_BYTES  HT20_NUM_SUBCARRIERS   /* one uint8 magnitude per subcarrier */

void CalibrationFileBuffer::init(const char* buffer_path, uint16_t buffer_size) {
  buffer_path_ = buffer_path;
  buffer_size_ = buffer_size;
  ESP_LOGD(TAG, "Initialized (RAM, size: %d packets = %d bytes)",
           buffer_size_, buffer_size_ * PKT_BYTES);
}

void CalibrationFileBuffer::reset() {
  buffer_count_ = 0;
  read_pos_ = 0;
  last_progress_ = 0;
}

bool CalibrationFileBuffer::open_for_writing() {
  if (!ram_) {
    ram_ = (uint8_t*) heap_caps_malloc((size_t)buffer_size_ * PKT_BYTES, MALLOC_CAP_8BIT);
    if (!ram_) {
      ESP_LOGE(TAG, "Failed to allocate %d-byte calibration buffer", buffer_size_ * PKT_BYTES);
      return false;
    }
  }
  buffer_count_ = 0;
  read_pos_ = 0;
  open_ = true;
  return true;
}

bool CalibrationFileBuffer::open_for_reading() {
  if (!ram_) {
    ESP_LOGE(TAG, "No calibration buffer to read");
    return false;
  }
  read_pos_ = 0;
  open_ = true;
  return true;
}

void CalibrationFileBuffer::close() {
  open_ = false;   /* keep ram_ so a write→read cycle can read it back */
}

void CalibrationFileBuffer::remove_file() {
  if (ram_) {
    heap_caps_free(ram_);
    ram_ = nullptr;
  }
  buffer_count_ = 0;
  read_pos_ = 0;
}

bool CalibrationFileBuffer::write_packet(const int8_t* csi_data, size_t csi_len) {
  if (is_full() || !ram_ || !open_) {
    return is_full();
  }
  uint16_t packet_sc = csi_len / 2;
  if (packet_sc != HT20_NUM_SUBCARRIERS) {
    return false;
  }

  uint8_t* dst = ram_ + (size_t)buffer_count_ * PKT_BYTES;
  for (uint16_t sc = 0; sc < HT20_NUM_SUBCARRIERS; sc++) {
    if (sc < HT20_GUARD_BAND_LOW || sc > HT20_GUARD_BAND_HIGH || sc == HT20_DC_SUBCARRIER) {
      dst[sc] = 0;
      continue;
    }
    int8_t q_val = csi_data[sc * 2];      // Imaginary first
    int8_t i_val = csi_data[sc * 2 + 1];  // Real second
    float mag = calculate_magnitude(i_val, q_val);
    dst[sc] = static_cast<uint8_t>(std::min(mag, 255.0f));
  }
  buffer_count_++;

  uint8_t progress = (buffer_count_ * 100) / buffer_size_;
  if (progress >= last_progress_ + 10 || buffer_count_ == buffer_size_) {
    log_progress_bar(TAG, progress / 100.0f, 20, -1,
                     "%d%% (%d/%d)", progress, buffer_count_, buffer_size_);
    last_progress_ = progress;
  }
  return is_full();
}

std::vector<uint8_t> CalibrationFileBuffer::read_window(uint16_t start_idx, uint16_t window_size) {
  std::vector<uint8_t> data;
  if (!ram_) {
    ESP_LOGE(TAG, "No calibration buffer to read");
    return data;
  }
  if (start_idx >= buffer_count_) {
    return data;
  }
  uint16_t avail = buffer_count_ - start_idx;
  if (window_size > avail) {
    window_size = avail;
  }
  size_t bytes = (size_t)window_size * PKT_BYTES;
  data.resize(bytes);
  memcpy(data.data(), ram_ + (size_t)start_idx * PKT_BYTES, bytes);
  return data;
}

}  // namespace espectre
}  // namespace esphome
