/**
 * @file hx711.c
 * @brief HX711 24-bit ADC Load Cell Amplifier Driver Implementation
 *
 * HX711 Communication Timing:
 * ==========================
 *
 * The HX711 uses a custom serial protocol (not SPI/I2C):
 *
 *     DOUT: ──────┐                              ┌───────
 *                 │         24 data bits         │
 *                 └──────────────────────────────┘
 *
 *     SCK:  ─┐ ┌┐ ┌┐ ┌┐ ┌┐ ┌┐ ┌┐ ... (24 or 25-27 pulses)
 *            │ ││ ││ ││ ││ ││ ││
 *            └─┘└─┘└─┘└─┘└─┘└─┘└─ ...
 *
 * Timing Requirements:
 * - SCK high time: min 0.2us, max 50us
 * - SCK low time: min 0.2us
 * - After 60us of SCK high, HX711 enters power-down mode
 *
 * Data Format:
 * - MSB first, 24 bits, two's complement
 * - Additional 1-3 clock pulses set gain for NEXT reading:
 *   * 25 pulses = Channel A, Gain 128
 *   * 26 pulses = Channel B, Gain 32
 *   * 27 pulses = Channel A, Gain 64
 *
 * Calibration:
 * ===========
 * Weight = (raw_value - offset) / scale
 *
 * 1. Tare: Read with no load, store as 'offset'
 * 2. Span: Place known weight, scale = (raw - offset) / known_weight
 */

#include "hx711.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "hx711";

/*******************************************************************************
 * Private Definitions
 ******************************************************************************/

#define NVS_NAMESPACE "loadcell"
#define NVS_KEY_OFFSET "offset"
#define NVS_KEY_SCALE "scale"

#define MAX_FILTER_SIZE 50
#define MAX_MEDIAN_SIZE 9

#define HX711_TIMEOUT_MS 200              // Max wait for DOUT low
#define HX711_MIN_VALUE -8388608          // -2^23
#define HX711_MAX_VALUE 8388607           // 2^23 - 1
#define HX711_DISCONNECT_VALUE (-8000000) // Value indicating disconnection

/*******************************************************************************
 * Private Structures
 ******************************************************************************/

struct hx711_s {
  // GPIO configuration
  gpio_num_t gpio_data;
  gpio_num_t gpio_clock;
  hx711_gain_t gain;

  // Calibration
  int32_t offset;
  float scale;
  bool calibrated;

  // Filtering
  uint8_t filter_size;
  uint8_t median_size;
  int32_t filter_buffer[MAX_FILTER_SIZE];
  uint8_t filter_index;
  bool filter_filled;
  float ema_alpha;      // EMA smoothing factor (0.01-1.0, lower = smoother)
  bool ema_initialized; // First sample flag

  // Current values
  int32_t raw_value;
  float weight_kg;

  // Connection status
  int32_t last_values[10];
  uint8_t last_index;
  bool power_down;

  // Thread safety
  SemaphoreHandle_t mutex;
};

/*******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/

static int32_t hx711_shift_in(hx711_handle_t handle);
static int compare_int32(const void *a, const void *b);
static int32_t hx711_median_filter(int32_t *values, uint8_t size);
static int32_t hx711_moving_average(hx711_handle_t handle, int32_t new_value);

/*******************************************************************************
 * Public API Implementation
 ******************************************************************************/

hx711_config_t hx711_get_default_config(void) {
  hx711_config_t config = {
      .gpio_data = 6,
      .gpio_clock = 7,
      .gain = HX711_GAIN_128_CH_A,
      .filter_size = 20,
      .median_size = 5,
  };
  return config;
}

esp_err_t hx711_init(const hx711_config_t *config, hx711_handle_t *handle) {
  if (config == NULL || handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (config->filter_size < 1 || config->filter_size > MAX_FILTER_SIZE) {
    ESP_LOGE(TAG, "Filter size must be 1-%d", MAX_FILTER_SIZE);
    return ESP_ERR_INVALID_ARG;
  }

  if (config->median_size < 1 || config->median_size > MAX_MEDIAN_SIZE ||
      (config->median_size % 2) == 0) {
    ESP_LOGE(TAG, "Median size must be odd and 1-%d", MAX_MEDIAN_SIZE);
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI(TAG, "Initializing HX711: DATA=%d, CLK=%d, Gain=%d",
           config->gpio_data, config->gpio_clock, config->gain);

  // Allocate structure
  struct hx711_s *hx = calloc(1, sizeof(struct hx711_s));
  if (hx == NULL) {
    ESP_LOGE(TAG, "Failed to allocate HX711 structure");
    return ESP_ERR_NO_MEM;
  }

  // Store configuration
  hx->gpio_data = config->gpio_data;
  hx->gpio_clock = config->gpio_clock;
  hx->gain = config->gain;
  hx->filter_size = config->filter_size;
  hx->median_size = config->median_size;
  hx->scale = 1.0f; // Default scale (raw values)
#ifdef CONFIG_HX711_EMA_ALPHA
  hx->ema_alpha = CONFIG_HX711_EMA_ALPHA / 100.0f;
#else
  hx->ema_alpha = 0.1f;
#endif
  hx->ema_initialized = false;

  // Create mutex
  hx->mutex = xSemaphoreCreateMutex();
  if (hx->mutex == NULL) {
    ESP_LOGE(TAG, "Failed to create mutex");
    free(hx);
    return ESP_ERR_NO_MEM;
  }

  // Configure GPIOs
  gpio_config_t io_conf = {
      .intr_type = GPIO_INTR_DISABLE,
      .mode = GPIO_MODE_INPUT,
      .pin_bit_mask = (1ULL << config->gpio_data),
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .pull_up_en = GPIO_PULLUP_DISABLE,
  };
  gpio_config(&io_conf);

  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = (1ULL << config->gpio_clock);
  gpio_config(&io_conf);

  // Start with clock low (HX711 active)
  gpio_set_level(hx->gpio_clock, 0);

  // Try to do a test read
  vTaskDelay(pdMS_TO_TICKS(100)); // Wait for HX711 to stabilize

  if (hx711_wait_ready(hx, 500) != ESP_OK) {
    ESP_LOGW(TAG, "HX711 not responding - check wiring");
  }

  ESP_LOGI(TAG, "HX711 initialized successfully");

  *handle = hx;
  return ESP_OK;
}

esp_err_t hx711_deinit(hx711_handle_t handle) {
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  // Power down
  gpio_set_level(handle->gpio_clock, 1);

  vSemaphoreDelete(handle->mutex);
  free(handle);

  ESP_LOGI(TAG, "HX711 deinitialized");
  return ESP_OK;
}

bool hx711_is_ready(hx711_handle_t handle) {
  if (handle == NULL || handle->power_down) {
    return false;
  }
  return gpio_get_level(handle->gpio_data) == 0;
}

esp_err_t hx711_wait_ready(hx711_handle_t handle, uint32_t timeout_ms) {
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (handle->power_down) {
    hx711_power_up(handle);
  }

  int64_t start = esp_timer_get_time();
  int64_t timeout_us = timeout_ms * 1000;

  while (gpio_get_level(handle->gpio_data) != 0) {
    if ((esp_timer_get_time() - start) > timeout_us) {
      return ESP_ERR_TIMEOUT;
    }
    vTaskDelay(1); // Yield to other tasks
  }

  return ESP_OK;
}

esp_err_t hx711_read_raw(hx711_handle_t handle, int32_t *value) {
  if (handle == NULL || value == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  xSemaphoreTake(handle->mutex, portMAX_DELAY);

  // Wait for data ready
  if (hx711_wait_ready(handle, HX711_TIMEOUT_MS) != ESP_OK) {
    xSemaphoreGive(handle->mutex);
    ESP_LOGW(TAG, "HX711 timeout waiting for data");
    return ESP_ERR_TIMEOUT;
  }

  // Read 24 bits
  int32_t raw = hx711_shift_in(handle);

  // Store for disconnect detection
  handle->last_values[handle->last_index] = raw;
  handle->last_index = (handle->last_index + 1) % 10;

  handle->raw_value = raw;
  *value = raw;

  xSemaphoreGive(handle->mutex);
  return ESP_OK;
}

esp_err_t hx711_read_filtered(hx711_handle_t handle, int32_t *value) {
  if (handle == NULL || value == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  // Read multiple samples for median filter
  int32_t samples[MAX_MEDIAN_SIZE];

  for (uint8_t i = 0; i < handle->median_size; i++) {
    esp_err_t ret = hx711_read_raw(handle, &samples[i]);
    if (ret != ESP_OK) {
      return ret;
    }
    vTaskDelay(1); // Small delay between readings
  }

  // Apply median filter (removes spikes)
  int32_t median = hx711_median_filter(samples, handle->median_size);

  // Apply moving average filter
  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  int32_t filtered = hx711_moving_average(handle, median);
  xSemaphoreGive(handle->mutex);

  *value = filtered;
  return ESP_OK;
}

esp_err_t hx711_get_weight_kg(hx711_handle_t handle, float *weight) {
  if (handle == NULL || weight == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (!handle->calibrated) {
    ESP_LOGW(TAG, "HX711 not calibrated");
    return ESP_ERR_INVALID_STATE;
  }

  int32_t filtered;
  esp_err_t ret = hx711_read_filtered(handle, &filtered);
  if (ret != ESP_OK) {
    return ret;
  }

  // Apply calibration: weight = (raw - offset) / scale
  // Note: scale is in raw_counts per kg
  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  float w = (float)(filtered - handle->offset) / handle->scale;
  handle->weight_kg = w;
  xSemaphoreGive(handle->mutex);

  *weight = w;
  return ESP_OK;
}

float hx711_update(hx711_handle_t handle) {
  if (handle == NULL) {
    return 0.0f;
  }

  // Non-blocking check for data ready
  if (!hx711_is_ready(handle)) {
    return handle->weight_kg; // Return cached value
  }

  int32_t raw;
  if (hx711_read_raw(handle, &raw) != ESP_OK) {
    return handle->weight_kg;
  }

  xSemaphoreTake(handle->mutex, portMAX_DELAY);

  // Apply moving average on raw values
  int32_t filtered = hx711_moving_average(handle, raw);

  // Calculate instantaneous weight
  float instant_weight;
  if (handle->calibrated && handle->scale != 0) {
    instant_weight = (float)(filtered - handle->offset) / handle->scale;
  } else if (handle->offset != 0) {
    instant_weight = (float)(filtered - handle->offset) / 1000.0f;
  } else {
    instant_weight = (float)filtered / 100000.0f;
  }

  // Apply EMA (Exponential Moving Average) for smooth output
  if (!handle->ema_initialized) {
    handle->weight_kg = instant_weight;
    handle->ema_initialized = true;
  } else {
    handle->weight_kg +=
        handle->ema_alpha * (instant_weight - handle->weight_kg);
  }

  float result = handle->weight_kg;
  xSemaphoreGive(handle->mutex);

  return result;
}

esp_err_t hx711_tare(hx711_handle_t handle, uint8_t samples) {
  if (handle == NULL || samples < 1 || samples > 100) {
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI(TAG, "Starting tare calibration with %d samples", samples);

  int64_t sum = 0;

  for (uint8_t i = 0; i < samples; i++) {
    int32_t value;
    esp_err_t ret = hx711_read_raw(handle, &value);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Tare failed to read sample %d", i);
      return ret;
    }
    sum += value;
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  handle->offset = (int32_t)(sum / samples);
  xSemaphoreGive(handle->mutex);

  ESP_LOGI(TAG, "Tare complete: offset = %ld", (long)handle->offset);

  // Save ONLY offset to NVS (don't touch scale)
  {
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
      nvs_set_i32(nvs, NVS_KEY_OFFSET, handle->offset);
      nvs_commit(nvs);
      nvs_close(nvs);
      ESP_LOGI(TAG, "Tare offset saved to NVS");
    }
  }

  return ESP_OK;
}

esp_err_t hx711_calibrate_scale(hx711_handle_t handle, float known_weight_kg,
                                uint8_t samples) {
  if (handle == NULL || known_weight_kg <= 0 || samples < 1 || samples > 100) {
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI(TAG, "Calibrating scale with %.3f kg", known_weight_kg);

  int64_t sum = 0;

  for (uint8_t i = 0; i < samples; i++) {
    int32_t value;
    esp_err_t ret = hx711_read_raw(handle, &value);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Calibration failed to read sample %d", i);
      return ret;
    }
    sum += value;
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  int32_t average = (int32_t)(sum / samples);

  xSemaphoreTake(handle->mutex, portMAX_DELAY);

  // scale = (raw - offset) / weight
  // So: weight = (raw - offset) / scale
  float delta = (float)(average - handle->offset);

  if (delta == 0) {
    ESP_LOGE(TAG, "Cannot calibrate - no difference from tare");
    xSemaphoreGive(handle->mutex);
    return ESP_ERR_INVALID_STATE;
  }

  handle->scale = delta / known_weight_kg;
  handle->calibrated = true;

  ESP_LOGI(TAG, "Calibration complete: scale = %.2f counts/kg", handle->scale);
  ESP_LOGI(TAG, "Offset=%ld, Scale=%.2f", (long)handle->offset, handle->scale);

  xSemaphoreGive(handle->mutex);

  return ESP_OK;
}

esp_err_t hx711_set_calibration(hx711_handle_t handle, int32_t offset,
                                float scale) {
  if (handle == NULL || scale == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  handle->offset = offset;
  handle->scale = scale;
  handle->calibrated = true;
  xSemaphoreGive(handle->mutex);

  ESP_LOGI(TAG, "Calibration set: offset=%ld, scale=%.2f", (long)offset, scale);

  return ESP_OK;
}

esp_err_t hx711_get_calibration(hx711_handle_t handle,
                                hx711_calibration_t *calibration) {
  if (handle == NULL || calibration == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  calibration->offset = handle->offset;
  calibration->scale = handle->scale;
  calibration->valid = handle->calibrated;
  xSemaphoreGive(handle->mutex);

  return ESP_OK;
}

esp_err_t hx711_save_calibration(hx711_handle_t handle) {
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  nvs_handle_t nvs;
  esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
    return ret;
  }

  xSemaphoreTake(handle->mutex, portMAX_DELAY);

  ret = nvs_set_i32(nvs, NVS_KEY_OFFSET, handle->offset);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to save offset: %s", esp_err_to_name(ret));
    nvs_close(nvs);
    xSemaphoreGive(handle->mutex);
    return ret;
  }

  // Store float as bytes
  ret = nvs_set_blob(nvs, NVS_KEY_SCALE, &handle->scale, sizeof(float));
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to save scale: %s", esp_err_to_name(ret));
    nvs_close(nvs);
    xSemaphoreGive(handle->mutex);
    return ret;
  }

  xSemaphoreGive(handle->mutex);

  ret = nvs_commit(nvs);
  nvs_close(nvs);

  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Calibration saved to NVS");
  }

  return ret;
}

esp_err_t hx711_load_calibration(hx711_handle_t handle) {
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  nvs_handle_t nvs;
  esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs);
  if (ret != ESP_OK) {
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
      ESP_LOGW(TAG, "No calibration found in NVS");
    }
    return ret;
  }

  // Load offset and scale INDEPENDENTLY
  // (tare only saves offset, so scale may not exist yet)
  bool has_offset = false, has_scale = false;
  int32_t offset = 0;
  float scale = 1.0f;

  if (nvs_get_i32(nvs, NVS_KEY_OFFSET, &offset) == ESP_OK) {
    has_offset = true;
  }

  size_t size = sizeof(float);
  if (nvs_get_blob(nvs, NVS_KEY_SCALE, &scale, &size) == ESP_OK) {
    has_scale = true;
  }

  nvs_close(nvs);

  if (!has_offset && !has_scale) {
    ESP_LOGW(TAG, "No calibration data in NVS");
    return ESP_ERR_NVS_NOT_FOUND;
  }

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  if (has_offset) {
    handle->offset = offset;
  }
  if (has_scale && scale != 0.0f && scale != 1.0f) {
    handle->scale = scale;
    handle->calibrated = true;
  }
  xSemaphoreGive(handle->mutex);

  ESP_LOGI(TAG,
           "Calibration loaded: offset=%ld (found=%d), scale=%.6f (found=%d), "
           "calibrated=%d",
           (long)offset, has_offset, scale, has_scale, handle->calibrated);

  return ESP_OK;
}

bool hx711_is_connected(hx711_handle_t handle) {
  if (handle == NULL) {
    return false;
  }

  xSemaphoreTake(handle->mutex, portMAX_DELAY);

  // Check if all recent values are the same (stuck)
  bool all_same = true;
  int32_t first = handle->last_values[0];
  for (int i = 1; i < 10; i++) {
    if (handle->last_values[i] != first) {
      all_same = false;
      break;
    }
  }

  // Check if values are out of range
  bool out_of_range = (handle->raw_value <= HX711_DISCONNECT_VALUE ||
                       handle->raw_value >= HX711_MAX_VALUE);

  xSemaphoreGive(handle->mutex);

  return !all_same && !out_of_range;
}

esp_err_t hx711_power_down(hx711_handle_t handle) {
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  // Hold SCK high for >60us to enter power-down mode
  gpio_set_level(handle->gpio_clock, 1);
  esp_rom_delay_us(100);
  handle->power_down = true;

  ESP_LOGI(TAG, "HX711 powered down");
  return ESP_OK;
}

esp_err_t hx711_power_up(hx711_handle_t handle) {
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  gpio_set_level(handle->gpio_clock, 0);
  handle->power_down = false;

  ESP_LOGI(TAG, "HX711 powered up");
  return ESP_OK;
}

/*******************************************************************************
 * Private Function Implementations
 ******************************************************************************/

/**
 * @brief Shift in 24 bits from HX711
 *
 * Uses bit-banging with precise timing. The timing is critical:
 * - SCK high time must be < 60us (or HX711 powers down)
 * - SCK period should be ~2us for reliable operation
 *
 * Additional clock pulses after 24 bits set the gain for the next reading.
 */
static int32_t hx711_shift_in(hx711_handle_t handle) {
  int32_t value = 0;
  uint8_t clock_pulses = 24 + (uint8_t)handle->gain; // 25, 26, or 27 pulses

  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

  // Critical section - timing sensitive
  taskENTER_CRITICAL(&mux);

  for (uint8_t i = 0; i < clock_pulses; i++) {
    // Clock high
    gpio_set_level(handle->gpio_clock, 1);
    esp_rom_delay_us(1);

    // Read data bit (only first 24 pulses)
    if (i < 24) {
      value = (value << 1) | gpio_get_level(handle->gpio_data);
    }

    // Clock low
    gpio_set_level(handle->gpio_clock, 0);
    esp_rom_delay_us(1);
  }

  taskEXIT_CRITICAL(&mux);

  // Convert 24-bit two's complement to 32-bit signed
  if (value & 0x800000) {
    value |= 0xFF000000; // Sign extend
  }

  return value;
}

/**
 * @brief Compare function for qsort
 */
static int compare_int32(const void *a, const void *b) {
  int32_t va = *(const int32_t *)a;
  int32_t vb = *(const int32_t *)b;
  return (va > vb) - (va < vb);
}

/**
 * @brief Apply median filter to remove spikes
 *
 * Sorts the values and returns the middle value.
 * Effective at removing occasional extreme readings.
 */
static int32_t hx711_median_filter(int32_t *values, uint8_t size) {
  // Copy to avoid modifying original array
  int32_t sorted[MAX_MEDIAN_SIZE];
  memcpy(sorted, values, size * sizeof(int32_t));

  // Sort
  qsort(sorted, size, sizeof(int32_t), compare_int32);

  // Return middle value
  return sorted[size / 2];
}

/**
 * @brief Apply moving average filter
 *
 * Maintains a circular buffer and calculates the running average.
 * Provides smooth readings while maintaining responsiveness.
 */
static int32_t hx711_moving_average(hx711_handle_t handle, int32_t new_value) {
  // Add to circular buffer
  handle->filter_buffer[handle->filter_index] = new_value;
  handle->filter_index = (handle->filter_index + 1) % handle->filter_size;

  if (handle->filter_index == 0) {
    handle->filter_filled = true;
  }

  // Calculate average
  int64_t sum = 0;
  uint8_t count =
      handle->filter_filled ? handle->filter_size : handle->filter_index;

  for (uint8_t i = 0; i < count; i++) {
    sum += handle->filter_buffer[i];
  }

  return (count > 0) ? (int32_t)(sum / count) : new_value;
}

void hx711_set_ema_alpha(hx711_handle_t handle, float alpha) {
  if (handle == NULL)
    return;
  if (alpha < 0.01f)
    alpha = 0.01f;
  if (alpha > 1.0f)
    alpha = 1.0f;

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  handle->ema_alpha = alpha;
  xSemaphoreGive(handle->mutex);

  ESP_LOGI(TAG, "EMA alpha set to %.2f", alpha);
}

void hx711_set_filter_size(hx711_handle_t handle, uint8_t size) {
  if (handle == NULL)
    return;
  if (size < 1)
    size = 1;
  if (size > MAX_FILTER_SIZE)
    size = MAX_FILTER_SIZE;

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  // Only change size - let the circular buffer adapt naturally
  // Don't reset filter_index or filter_filled to avoid noise spike
  handle->filter_size = size;
  if (handle->filter_index >= size) {
    handle->filter_index = 0;
  }
  xSemaphoreGive(handle->mutex);

  ESP_LOGI(TAG, "Filter size set to %d", size);
}
