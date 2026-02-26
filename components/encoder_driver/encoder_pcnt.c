/**
 * @file encoder_pcnt.c
 * @brief Quadrature Encoder Driver Implementation using ESP32-S3 PCNT
 *
 * PCNT Quadrature Decoding Principle:
 * ==================================
 * In 4x quadrature mode, we count all edges on both channels:
 *
 * Channel A: ──┐    ┌────┐    ┌────
 *              │    │    │    │
 *              └────┘    └────┘
 *
 * Channel B: ────┐    ┌────┐    ┌──
 *                │    │    │    │
 *                └────┘    └────┘
 *
 * For clockwise rotation:
 * - When A rises and B is low  → count up
 * - When A falls and B is high → count up
 * - When B rises and A is high → count up
 * - When B falls and A is low  → count up
 *
 * This gives 4 counts per encoder pulse (4x multiplication).
 * For a 600 PPR encoder: 600 * 4 = 2400 counts per revolution.
 *
 * RPM Calculation:
 * ===============
 * RPM = (delta_count / CPR) * (60 seconds / delta_time_seconds)
 * RPM = (delta_count / (PPR * 4)) * (60,000,000 / delta_time_us)
 *
 * Overflow Handling:
 * =================
 * The PCNT counter is 16-bit signed (-32768 to +32767).
 * We use watch points at ±1000 to trigger ISR before overflow.
 * The ISR accumulates counts into a 32-bit variable.
 */

#include "encoder_pcnt.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"
#include <math.h>
#include <string.h>


static const char *TAG = "encoder";

/*******************************************************************************
 * Private Definitions
 ******************************************************************************/

#define PCNT_HIGH_LIMIT 1000 // Watch point for overflow
#define PCNT_LOW_LIMIT -1000 // Watch point for underflow
#define MAX_FILTER_SIZE 20
#define US_PER_MINUTE 60000000ULL

/*******************************************************************************
 * Private Structures
 ******************************************************************************/

/**
 * @brief Internal encoder state structure
 */
struct encoder_s {
  // PCNT handles
  pcnt_unit_handle_t pcnt_unit;
  pcnt_channel_handle_t pcnt_chan_a;
  pcnt_channel_handle_t pcnt_chan_b;

  // Configuration
  uint32_t ppr; // Pulses per revolution
  uint32_t cpr; // Counts per revolution (PPR * 4)
  uint8_t filter_size;

  // Count accumulation
  volatile int32_t overflow_count; // Accumulated overflow counts
  volatile int32_t total_count;    // Total count (overflow + current)

  // RPM calculation
  int32_t prev_count;   // Previous count for delta calculation
  int64_t prev_time_us; // Previous timestamp
  float rpm_raw;        // Unfiltered RPM
  float rpm_filtered;   // Filtered RPM

  // RPM filter (moving average)
  float rpm_buffer[MAX_FILTER_SIZE];
  uint8_t filter_index;
  bool filter_filled; // True when buffer is full (for accurate average)

  // Direction tracking
  encoder_direction_t direction;
  int64_t last_pulse_time_us;

  // Thread safety
  SemaphoreHandle_t mutex;
};

/*******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/

static bool IRAM_ATTR encoder_pcnt_overflow_callback(
    pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata,
    void *user_ctx);
static float encoder_apply_filter(encoder_handle_t handle, float new_rpm);

/*******************************************************************************
 * Public API Implementation
 ******************************************************************************/

encoder_config_t encoder_get_default_config(void) {
  encoder_config_t config = {
      .gpio_a = 4,               // Default GPIO for channel A
      .gpio_b = 5,               // Default GPIO for channel B
      .ppr = CONFIG_ENCODER_PPR, // From Kconfig (single source of truth)
      .filter_size = 5,          // 5-sample moving average
      .count_high = PCNT_HIGH_LIMIT,
      .count_low = PCNT_LOW_LIMIT};
  return config;
}

esp_err_t encoder_init(const encoder_config_t *config,
                       encoder_handle_t *handle) {
  if (config == NULL || handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (config->filter_size < 1 || config->filter_size > MAX_FILTER_SIZE) {
    ESP_LOGE(TAG, "Filter size must be 1-%d", MAX_FILTER_SIZE);
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI(TAG, "Initializing encoder: GPIO_A=%d, GPIO_B=%d, PPR=%lu",
           config->gpio_a, config->gpio_b, (unsigned long)config->ppr);

  // Allocate encoder structure
  struct encoder_s *enc = calloc(1, sizeof(struct encoder_s));
  if (enc == NULL) {
    ESP_LOGE(TAG, "Failed to allocate encoder structure");
    return ESP_ERR_NO_MEM;
  }

  // Initialize configuration
  enc->ppr = config->ppr;
  enc->cpr = config->ppr * 4; // 4x quadrature
  enc->filter_size = config->filter_size;
  enc->prev_time_us = esp_timer_get_time();
  enc->last_pulse_time_us = enc->prev_time_us;

  // Create mutex for thread safety
  enc->mutex = xSemaphoreCreateMutex();
  if (enc->mutex == NULL) {
    ESP_LOGE(TAG, "Failed to create mutex");
    free(enc);
    return ESP_ERR_NO_MEM;
  }

  // Configure PCNT unit
  pcnt_unit_config_t unit_config = {
      .high_limit = config->count_high,
      .low_limit = config->count_low,
      .intr_priority = 0,
      .flags.accum_count = true, // Enable accumulation mode
  };

  esp_err_t ret = pcnt_new_unit(&unit_config, &enc->pcnt_unit);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create PCNT unit: %s", esp_err_to_name(ret));
    vSemaphoreDelete(enc->mutex);
    free(enc);
    return ret;
  }

  // Configure glitch filter (1us filter for noise rejection)
  pcnt_glitch_filter_config_t filter_config = {
      .max_glitch_ns = 1000, // 1 microsecond
  };
  pcnt_unit_set_glitch_filter(enc->pcnt_unit, &filter_config);

  /*
   * Configure Channel A:
   * - Edge action: count on both rising and falling edges
   * - Level action: use Channel B state to determine direction
   *
   * When A has edge and B is low  → count direction depends on edge
   * When A has edge and B is high → count opposite direction
   */
  pcnt_chan_config_t chan_a_config = {
      .edge_gpio_num = config->gpio_a,
      .level_gpio_num = config->gpio_b,
      .flags.invert_edge_input = false,
      .flags.invert_level_input = false,
      .flags.io_loop_back = false,
  };

  ret = pcnt_new_channel(enc->pcnt_unit, &chan_a_config, &enc->pcnt_chan_a);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create PCNT channel A: %s", esp_err_to_name(ret));
    pcnt_del_unit(enc->pcnt_unit);
    vSemaphoreDelete(enc->mutex);
    free(enc);
    return ret;
  }

  // Set channel A edge and level actions for quadrature decoding
  pcnt_channel_set_edge_action(
      enc->pcnt_chan_a,
      PCNT_CHANNEL_EDGE_ACTION_DECREASE,  // Rising edge: count based on B level
      PCNT_CHANNEL_EDGE_ACTION_INCREASE); // Falling edge: count opposite
  pcnt_channel_set_level_action(
      enc->pcnt_chan_a,
      PCNT_CHANNEL_LEVEL_ACTION_KEEP,     // When B is high, keep direction
      PCNT_CHANNEL_LEVEL_ACTION_INVERSE); // When B is low, inverse direction

  /*
   * Configure Channel B:
   * - Similar to Channel A but swapped roles
   * - Provides additional 2x counting for full 4x quadrature
   */
  pcnt_chan_config_t chan_b_config = {
      .edge_gpio_num = config->gpio_b,
      .level_gpio_num = config->gpio_a,
      .flags.invert_edge_input = false,
      .flags.invert_level_input = false,
      .flags.io_loop_back = false,
  };

  ret = pcnt_new_channel(enc->pcnt_unit, &chan_b_config, &enc->pcnt_chan_b);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create PCNT channel B: %s", esp_err_to_name(ret));
    pcnt_del_channel(enc->pcnt_chan_a);
    pcnt_del_unit(enc->pcnt_unit);
    vSemaphoreDelete(enc->mutex);
    free(enc);
    return ret;
  }

  // Set channel B edge and level actions
  pcnt_channel_set_edge_action(enc->pcnt_chan_b,
                               PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                               PCNT_CHANNEL_EDGE_ACTION_DECREASE);
  pcnt_channel_set_level_action(enc->pcnt_chan_b,
                                PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

  // Add watch points for overflow/underflow detection
  pcnt_unit_add_watch_point(enc->pcnt_unit, config->count_high);
  pcnt_unit_add_watch_point(enc->pcnt_unit, config->count_low);

  // Register overflow callback
  pcnt_event_callbacks_t callbacks = {
      .on_reach = encoder_pcnt_overflow_callback,
  };
  pcnt_unit_register_event_callbacks(enc->pcnt_unit, &callbacks, enc);

  // Enable and start PCNT unit
  pcnt_unit_enable(enc->pcnt_unit);
  pcnt_unit_clear_count(enc->pcnt_unit);
  pcnt_unit_start(enc->pcnt_unit);

  ESP_LOGI(TAG, "Encoder initialized: CPR=%lu (4x quadrature)",
           (unsigned long)enc->cpr);

  *handle = enc;
  return ESP_OK;
}

esp_err_t encoder_deinit(encoder_handle_t handle) {
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  pcnt_unit_stop(handle->pcnt_unit);
  pcnt_unit_disable(handle->pcnt_unit);
  pcnt_del_channel(handle->pcnt_chan_b);
  pcnt_del_channel(handle->pcnt_chan_a);
  pcnt_del_unit(handle->pcnt_unit);
  vSemaphoreDelete(handle->mutex);
  free(handle);

  ESP_LOGI(TAG, "Encoder deinitialized");
  return ESP_OK;
}

int32_t encoder_get_count(encoder_handle_t handle) {
  if (handle == NULL) {
    return 0;
  }

  int pcnt_count = 0;
  pcnt_unit_get_count(handle->pcnt_unit, &pcnt_count);

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  int32_t total = handle->overflow_count + pcnt_count;
  xSemaphoreGive(handle->mutex);

  return total;
}

esp_err_t encoder_reset_count(encoder_handle_t handle) {
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  xSemaphoreTake(handle->mutex, portMAX_DELAY);

  pcnt_unit_clear_count(handle->pcnt_unit);
  handle->overflow_count = 0;
  handle->total_count = 0;
  handle->prev_count = 0;

  // Reset filter
  memset(handle->rpm_buffer, 0, sizeof(handle->rpm_buffer));
  handle->filter_index = 0;
  handle->filter_filled = false;
  handle->rpm_raw = 0;
  handle->rpm_filtered = 0;

  xSemaphoreGive(handle->mutex);

  ESP_LOGI(TAG, "Encoder count reset");
  return ESP_OK;
}

esp_err_t encoder_update(encoder_handle_t handle) {
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  int64_t now_us = esp_timer_get_time();
  int32_t current_count = encoder_get_count(handle);

  xSemaphoreTake(handle->mutex, portMAX_DELAY);

  // Calculate delta count and time
  int32_t delta_count = current_count - handle->prev_count;
  int64_t delta_time_us = now_us - handle->prev_time_us;

  // Prevent division by zero and handle wrap-around
  if (delta_time_us > 0 && delta_time_us < 1000000) { // Max 1 second
    /*
     * RPM Calculation:
     * RPM = (counts / counts_per_rev) * (60 sec/min) * (1000000 us/sec) /
     * delta_time_us RPM = (delta_count / CPR) * (60,000,000 / delta_time_us)
     */
    float rpm = ((float)delta_count / (float)handle->cpr) *
                ((float)US_PER_MINUTE / (float)delta_time_us);

    handle->rpm_raw = rpm;
    handle->rpm_filtered = encoder_apply_filter(handle, rpm);

    // Determine direction
    if (delta_count > 0) {
      handle->direction = ENCODER_DIR_CW;
      handle->last_pulse_time_us = now_us;
    } else if (delta_count < 0) {
      handle->direction = ENCODER_DIR_CCW;
      handle->last_pulse_time_us = now_us;
    } else {
      // No movement - check if stopped
      if ((now_us - handle->last_pulse_time_us) > 100000) { // 100ms timeout
        handle->direction = ENCODER_DIR_STOPPED;
      }
    }
  }

  // Update previous values for next calculation
  handle->prev_count = current_count;
  handle->prev_time_us = now_us;
  handle->total_count = current_count;

  xSemaphoreGive(handle->mutex);

  return ESP_OK;
}

float encoder_get_rpm(encoder_handle_t handle) {
  if (handle == NULL) {
    return 0.0f;
  }

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  float rpm = handle->rpm_filtered;
  xSemaphoreGive(handle->mutex);

  return rpm;
}

float encoder_get_rpm_raw(encoder_handle_t handle) {
  if (handle == NULL) {
    return 0.0f;
  }

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  float rpm = handle->rpm_raw;
  xSemaphoreGive(handle->mutex);

  return rpm;
}

encoder_direction_t encoder_get_direction(encoder_handle_t handle) {
  if (handle == NULL) {
    return ENCODER_DIR_STOPPED;
  }

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  encoder_direction_t dir = handle->direction;
  xSemaphoreGive(handle->mutex);

  return dir;
}

bool encoder_is_active(encoder_handle_t handle, uint32_t timeout_ms) {
  return encoder_get_idle_time_ms(handle) < timeout_ms;
}

uint32_t encoder_get_idle_time_ms(encoder_handle_t handle) {
  if (handle == NULL) {
    return UINT32_MAX;
  }

  int64_t now_us = esp_timer_get_time();

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  int64_t idle_us = now_us - handle->last_pulse_time_us;
  xSemaphoreGive(handle->mutex);

  return (uint32_t)(idle_us / 1000);
}

/*******************************************************************************
 * Private Function Implementations
 ******************************************************************************/

/**
 * @brief PCNT overflow/underflow callback (runs in ISR context)
 *
 * When the counter reaches the high or low watch point, this callback
 * is triggered. We accumulate the count and clear the counter.
 */
static bool IRAM_ATTR encoder_pcnt_overflow_callback(
    pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata,
    void *user_ctx) {
  struct encoder_s *enc = (struct encoder_s *)user_ctx;
  BaseType_t high_task_wakeup = pdFALSE;

  // Accumulate the watch point value (high or low limit)
  enc->overflow_count += edata->watch_point_value;
  enc->last_pulse_time_us = esp_timer_get_time();

  // Note: PCNT counter is automatically cleared after reaching watch point
  // when using accumulation mode

  return high_task_wakeup == pdTRUE;
}

/**
 * @brief Apply moving average filter to RPM value
 *
 * Uses a circular buffer to maintain the last N samples and calculates
 * the average. This reduces noise and provides smooth RPM readings.
 *
 * @param handle    Encoder handle
 * @param new_rpm   New RPM sample to add
 * @return Filtered RPM value
 */
static float encoder_apply_filter(encoder_handle_t handle, float new_rpm) {
  // Add new sample to circular buffer
  handle->rpm_buffer[handle->filter_index] = new_rpm;
  handle->filter_index = (handle->filter_index + 1) % handle->filter_size;

  if (handle->filter_index == 0) {
    handle->filter_filled = true; // Buffer has wrapped around once
  }

  // Calculate average
  float sum = 0.0f;
  uint8_t count =
      handle->filter_filled ? handle->filter_size : handle->filter_index;

  for (uint8_t i = 0; i < count; i++) {
    sum += handle->rpm_buffer[i];
  }

  return (count > 0) ? (sum / (float)count) : 0.0f;
}
