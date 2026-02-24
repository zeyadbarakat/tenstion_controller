/**
 * @file relay_feedback.c
 * @brief Relay Feedback Auto-Tuning Implementation
 *
 * Relay Feedback Theory (Åström-Hägglund, 1984):
 * =============================================
 *
 * The relay feedback test induces a controlled oscillation:
 *
 *   Output:    d ────┐         ┌─────┐         ┌─────
 *                    │         │     │         │
 *              0 ────┴─────────┴     └─────────┴
 *
 *   PV:           /\    /\    /\    /\    /\
 *              ──/  \──/  \──/  \──/  \──/  \──
 *
 * Where:
 *   d = relay amplitude (output swing)
 *   a = process variable oscillation amplitude
 *   Tu = oscillation period (ultimate period)
 *
 * The ultimate gain is:
 *   Ku = (4 * d) / (π * a)
 *
 * This is based on the describing function of a relay,
 * which approximates the fundamental component of the relay output.
 *
 * Ziegler-Nichols Tuning Rules:
 * ============================
 *
 * For PI controller:
 *   Kp = 0.45 * Ku
 *   Ki = 1.2 * Kp / Tu = 0.54 * Ku / Tu
 *
 * Alternative rules:
 * - Tyreus-Luyben (more conservative): Kp = 0.31 * Ku
 * - Some overshoot: Kp = 0.33 * Ku
 * - No overshoot: Kp = 0.20 * Ku
 *
 * Peak Detection Algorithm:
 * ========================
 * 1. Track when PV crosses setpoint
 * 2. Between crossings, find max/min values
 * 3. These are the peaks and valleys
 * 4. Average multiple cycles for accuracy
 */

#include "relay_feedback.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <math.h>
#include <string.h>

static const char *TAG = "autotune";

/*******************************************************************************
 * Private Definitions
 ******************************************************************************/

#define MAX_PEAKS 20
#define HYSTERESIS_PERCENT 2.0f // Hysteresis band around setpoint
#define NVS_NAMESPACE "autotune"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/*******************************************************************************
 * Private Structures
 ******************************************************************************/

struct autotune_s {
  // Configuration
  autotune_config_t config;

  // State
  autotune_state_t state;
  int64_t start_time_us;
  int64_t last_crossing_us;

  // Relay state
  bool relay_high; // Current relay output state
  float current_output;

  // Peak detection
  float peaks[MAX_PEAKS];
  float valleys[MAX_PEAKS];
  int64_t peak_times[MAX_PEAKS];
  uint8_t peak_count;
  uint8_t valley_count;

  // Current cycle tracking
  float cycle_max;
  float cycle_min;
  bool above_setpoint;

  // Stable detection
  float prev_pv;
  uint32_t stable_count;

  // Results
  autotune_result_t result;

  // Status
  autotune_status_t status;
};

/*******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/

static void detect_peaks(autotune_handle_t handle, float pv, float setpoint);
static void analyze_oscillation(autotune_handle_t handle);
static void calculate_gains(autotune_handle_t handle);
static void update_status(autotune_handle_t handle, const char *msg);

/*******************************************************************************
 * Public API Implementation
 ******************************************************************************/

autotune_config_t autotune_get_default_config(void) {
  autotune_config_t config = {
      .loop = AUTOTUNE_SPEED_LOOP,
      .relay_amplitude = 20.0f, // ±20% output swing
      .bias = 50.0f,            // 50% bias
      .setpoint = 0.0f,         // Must be set by user
      .min_cycles = 3,          // Minimum 3 oscillation cycles
      .timeout_s = 120,         // 2 minute timeout
      .rule =
          TUNE_RULE_NO_OVERSHOOT, // Safe for unwinding (no material breakage)
  };
  return config;
}

esp_err_t autotune_init(autotune_handle_t *handle) {
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  struct autotune_s *at = calloc(1, sizeof(struct autotune_s));
  if (at == NULL) {
    return ESP_ERR_NO_MEM;
  }

  at->state = TUNE_IDLE;
  at->config = autotune_get_default_config();

  ESP_LOGI(TAG, "Auto-tuner initialized");

  *handle = at;
  return ESP_OK;
}

void autotune_deinit(autotune_handle_t handle) {
  if (handle) {
    free(handle);
  }
}

esp_err_t autotune_start(autotune_handle_t handle,
                         const autotune_config_t *config) {
  if (handle == NULL || config == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (config->relay_amplitude <= 0 || config->min_cycles < 1) {
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI(TAG, "Starting auto-tune: loop=%d, amplitude=%.1f, bias=%.1f",
           config->loop, config->relay_amplitude, config->bias);

  // Store configuration
  memcpy(&handle->config, config, sizeof(autotune_config_t));

  // Reset state
  handle->state = TUNE_INIT;
  handle->start_time_us = esp_timer_get_time();
  handle->last_crossing_us = handle->start_time_us;
  handle->relay_high = true;
  handle->current_output = config->bias + config->relay_amplitude;
  handle->peak_count = 0;
  handle->valley_count = 0;
  handle->cycle_max = -1e10f;
  handle->cycle_min = 1e10f;
  handle->above_setpoint = false;
  handle->stable_count = 0;
  memset(&handle->result, 0, sizeof(autotune_result_t));

  update_status(handle, "Initializing...");

  return ESP_OK;
}

float autotune_update(autotune_handle_t handle, float setpoint,
                      float measured_value) {
  if (handle == NULL) {
    return 0.0f;
  }

  if (handle->state == TUNE_IDLE || handle->state == TUNE_COMPLETE ||
      handle->state == TUNE_FAILED || handle->state == TUNE_ABORTED) {
    return 0.0f;
  }

  int64_t now = esp_timer_get_time();
  float elapsed_s = (float)(now - handle->start_time_us) / 1000000.0f;

  // Check timeout
  if (elapsed_s > handle->config.timeout_s) {
    ESP_LOGW(TAG, "Auto-tune timeout");
    handle->state = TUNE_FAILED;
    update_status(handle, "Timeout - no oscillation");
    return 0.0f;
  }

  // Update progress
  handle->status.progress_percent =
      (uint8_t)((elapsed_s / handle->config.timeout_s) * 100);
  handle->status.current_pv = measured_value;
  handle->status.current_output = handle->current_output;

  switch (handle->state) {
  case TUNE_INIT:
    // Wait a moment before starting
    if (elapsed_s > 1.0f) {
      handle->state = TUNE_WAITING_STABLE;
      update_status(handle, "Waiting for stable...");
    }
    break;

  case TUNE_WAITING_STABLE:
    // Wait for PV to stabilize
    if (fabsf(measured_value - handle->prev_pv) < 0.1f) {
      handle->stable_count++;
    } else {
      handle->stable_count = 0;
    }

    if (handle->stable_count > 10) { // Stable for 10 samples
      handle->state = TUNE_RELAY;
      handle->above_setpoint = (measured_value > setpoint);
      update_status(handle, "Relay test running...");
    }
    break;

  case TUNE_RELAY:
    // Apply relay control with hysteresis
    {
      float hysteresis = fabsf(setpoint) * HYSTERESIS_PERCENT / 100.0f;
      if (hysteresis < 0.1f)
        hysteresis = 0.1f;

      bool should_be_high = handle->relay_high;

      if (measured_value > setpoint + hysteresis) {
        should_be_high = false; // Above setpoint, reduce output
      } else if (measured_value < setpoint - hysteresis) {
        should_be_high = true; // Below setpoint, increase output
      }

      // Switch relay if needed
      if (should_be_high != handle->relay_high) {
        handle->relay_high = should_be_high;

        if (should_be_high) {
          handle->current_output =
              handle->config.bias + handle->config.relay_amplitude;
        } else {
          handle->current_output =
              handle->config.bias - handle->config.relay_amplitude;
        }

        ESP_LOGD(TAG, "Relay switched: %s, output=%.1f",
                 should_be_high ? "HIGH" : "LOW", handle->current_output);
      }

      // Detect peaks and valleys
      detect_peaks(handle, measured_value, setpoint);

      // Check if we have enough cycles
      uint8_t complete_cycles = (handle->peak_count > handle->valley_count)
                                    ? handle->valley_count
                                    : handle->peak_count;

      if (complete_cycles >= handle->config.min_cycles) {
        handle->state = TUNE_ANALYZING;
        update_status(handle, "Analyzing data...");
      }
    }
    break;

  case TUNE_ANALYZING:
    analyze_oscillation(handle);
    calculate_gains(handle);

    if (handle->result.valid) {
      handle->state = TUNE_COMPLETE;
      snprintf(handle->status.message, sizeof(handle->status.message),
               "Complete: Kp=%.3f Ki=%.3f", handle->result.kp,
               handle->result.ki);
      ESP_LOGI(TAG, "Auto-tune complete: Ku=%.3f, Tu=%.3f, Kp=%.3f, Ki=%.3f",
               handle->result.ku, handle->result.tu, handle->result.kp,
               handle->result.ki);
    } else {
      handle->state = TUNE_FAILED;
      update_status(handle, "Analysis failed");
    }
    return 0.0f;

  default:
    break;
  }

  handle->prev_pv = measured_value;
  return handle->current_output;
}

bool autotune_is_complete(autotune_handle_t handle) {
  if (handle == NULL)
    return true;
  return handle->state == TUNE_COMPLETE || handle->state == TUNE_FAILED ||
         handle->state == TUNE_ABORTED;
}

bool autotune_is_active(autotune_handle_t handle) {
  if (handle == NULL)
    return false;
  return handle->state != TUNE_IDLE && handle->state != TUNE_COMPLETE &&
         handle->state != TUNE_FAILED && handle->state != TUNE_ABORTED;
}

esp_err_t autotune_get_result(autotune_handle_t handle,
                              autotune_result_t *result) {
  if (handle == NULL || result == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (!handle->result.valid) {
    return ESP_ERR_INVALID_STATE;
  }

  memcpy(result, &handle->result, sizeof(autotune_result_t));
  return ESP_OK;
}

void autotune_get_status(autotune_handle_t handle, autotune_status_t *status) {
  if (handle == NULL || status == NULL) {
    return;
  }

  handle->status.state = handle->state;
  memcpy(status, &handle->status, sizeof(autotune_status_t));
}

esp_err_t autotune_apply_result(autotune_handle_t handle,
                                pi_controller_t *ctrl) {
  if (handle == NULL || ctrl == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (!handle->result.valid) {
    return ESP_ERR_INVALID_STATE;
  }

  pi_set_gains(ctrl, handle->result.kp, handle->result.ki);

  ESP_LOGI(TAG, "Applied tuning: Kp=%.3f, Ki=%.3f", handle->result.kp,
           handle->result.ki);

  return ESP_OK;
}

void autotune_abort(autotune_handle_t handle) {
  if (handle) {
    handle->state = TUNE_ABORTED;
    handle->current_output = 0.0f;
    update_status(handle, "Aborted by user");
    ESP_LOGW(TAG, "Auto-tune aborted");
  }
}

esp_err_t autotune_save_to_nvs(autotune_handle_t handle, autotune_loop_t loop) {
  if (handle == NULL || !handle->result.valid) {
    return ESP_ERR_INVALID_STATE;
  }

  nvs_handle_t nvs;
  esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
  if (ret != ESP_OK) {
    return ret;
  }

  // Create key based on loop type
  char key[16];
  snprintf(key, sizeof(key), "result_%d", loop);

  ret = nvs_set_blob(nvs, key, &handle->result, sizeof(autotune_result_t));
  if (ret == ESP_OK) {
    ret = nvs_commit(nvs);
    ESP_LOGI(TAG, "Saved tuning results for loop %d", loop);
  }

  nvs_close(nvs);
  return ret;
}

esp_err_t autotune_load_from_nvs(autotune_loop_t loop,
                                 autotune_result_t *result) {
  if (result == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  nvs_handle_t nvs;
  esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs);
  if (ret != ESP_OK) {
    return ret;
  }

  char key[16];
  snprintf(key, sizeof(key), "result_%d", loop);

  size_t size = sizeof(autotune_result_t);
  ret = nvs_get_blob(nvs, key, result, &size);

  nvs_close(nvs);

  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Loaded tuning results for loop %d: Kp=%.3f, Ki=%.3f", loop,
             result->kp, result->ki);
  }

  return ret;
}

/*******************************************************************************
 * Private Function Implementations
 ******************************************************************************/

/**
 * @brief Detect peaks and valleys in oscillation
 */
static void detect_peaks(autotune_handle_t handle, float pv, float setpoint) {
  // Track max/min in current cycle
  if (pv > handle->cycle_max) {
    handle->cycle_max = pv;
  }
  if (pv < handle->cycle_min) {
    handle->cycle_min = pv;
  }

  // Check for crossing setpoint
  bool now_above = (pv > setpoint);

  if (now_above != handle->above_setpoint) {
    int64_t now = esp_timer_get_time();

    if (handle->above_setpoint) {
      // Was above, now below - record peak
      if (handle->peak_count < MAX_PEAKS) {
        handle->peaks[handle->peak_count] = handle->cycle_max;
        handle->peak_times[handle->peak_count] = now;
        handle->peak_count++;
        ESP_LOGD(TAG, "Peak detected: %.2f", handle->cycle_max);
      }
      handle->cycle_max = pv; // Reset for next cycle
    } else {
      // Was below, now above - record valley
      if (handle->valley_count < MAX_PEAKS) {
        handle->valleys[handle->valley_count] = handle->cycle_min;
        handle->valley_count++;
        ESP_LOGD(TAG, "Valley detected: %.2f", handle->cycle_min);
      }
      handle->cycle_min = pv; // Reset for next cycle
    }

    handle->above_setpoint = now_above;
    handle->last_crossing_us = now;
  }
}

/**
 * @brief Analyze oscillation to get Ku and Tu
 */
static void analyze_oscillation(autotune_handle_t handle) {
  // Need at least 2 peaks for period calculation
  if (handle->peak_count < 2) {
    handle->result.valid = false;
    return;
  }

  // Calculate average period from peak times
  float total_period = 0;
  uint8_t period_count = 0;

  for (uint8_t i = 1; i < handle->peak_count; i++) {
    float period_s =
        (float)(handle->peak_times[i] - handle->peak_times[i - 1]) / 1000000.0f;
    if (period_s > 0.1f && period_s < 60.0f) { // Reasonable range
      total_period += period_s;
      period_count++;
    }
  }

  if (period_count == 0) {
    handle->result.valid = false;
    return;
  }

  float tu = total_period / period_count;

  // Calculate average oscillation amplitude
  float total_amp = 0;
  uint8_t amp_count = 0;
  uint8_t min_count = (handle->peak_count < handle->valley_count)
                          ? handle->peak_count
                          : handle->valley_count;

  for (uint8_t i = 0; i < min_count; i++) {
    float amp = handle->peaks[i] - handle->valleys[i];
    if (amp > 0) {
      total_amp += amp;
      amp_count++;
    }
  }

  if (amp_count == 0) {
    handle->result.valid = false;
    return;
  }

  float a = total_amp / amp_count / 2.0f; // Half amplitude (peak-to-center)

  // Validate oscillation amplitude - reject if too small (noise)
  if (a < 0.5f) {
    handle->result.valid = false;
    ESP_LOGW(TAG, "Oscillation amplitude too small: %.4f (min 0.5)", a * 2.0f);
    ESP_LOGW(TAG, "Check: relay amplitude, sensor connection, setpoint");
    return;
  }

  // Calculate ultimate gain: Ku = 4*d / (π*a)
  float d = handle->config.relay_amplitude;
  float ku = (4.0f * d) / (M_PI * a);

  // Validate Ku is in reasonable range
  if (ku < 0.01f || ku > 1000.0f) {
    handle->result.valid = false;
    ESP_LOGW(TAG, "Ultimate gain Ku=%.4f out of range (0.01-1000)", ku);
    return;
  }

  handle->result.tu = tu;
  handle->result.ku = ku;
  handle->result.oscillation_amp = a * 2.0f;
  handle->result.cycles_detected = min_count;

  ESP_LOGI(TAG, "Analysis: Tu=%.3fs, amplitude=%.2f, Ku=%.3f", tu, a * 2, ku);
}

/**
 * @brief Calculate PI gains from Ku and Tu
 */
static void calculate_gains(autotune_handle_t handle) {
  if (handle->result.ku <= 0 || handle->result.tu <= 0) {
    handle->result.valid = false;
    return;
  }

  float ku = handle->result.ku;
  float tu = handle->result.tu;
  float kp, ki;

  switch (handle->config.rule) {
  case TUNE_RULE_TYREUS_LUYBEN:
    kp = 0.31f * ku;
    ki = kp / (2.2f * tu);
    break;

  case TUNE_RULE_SOME_OVERSHOOT:
    kp = 0.33f * ku;
    ki = kp / (0.5f * tu);
    break;

  case TUNE_RULE_NO_OVERSHOOT:
    kp = 0.20f * ku;
    ki = kp / (0.5f * tu);
    break;

  case TUNE_RULE_ZIEGLER_NICHOLS:
  default:
    kp = 0.45f * ku;
    ki = 1.2f * kp / tu; // = 0.54 * Ku / Tu
    break;
  }

  // Sanity check: reject obviously bad gains
  if (kp < 0.001f || kp > 100.0f || ki < 0.0001f || ki > 50.0f) {
    handle->result.valid = false;
    ESP_LOGW(TAG, "Calculated gains out of range: Kp=%.6f Ki=%.6f", kp, ki);
    ESP_LOGW(TAG, "Expected: Kp 0.001-100.0, Ki 0.0001-50.0");
    return;
  }

  handle->result.kp = kp;
  handle->result.ki = ki;
  handle->result.valid = true;

  ESP_LOGI(TAG, "Calculated gains (rule %d): Kp=%.3f, Ki=%.3f",
           handle->config.rule, kp, ki);
}

/**
 * @brief Update status message
 */
static void update_status(autotune_handle_t handle, const char *msg) {
  strncpy(handle->status.message, msg, sizeof(handle->status.message) - 1);
  handle->status.message[sizeof(handle->status.message) - 1] = '\0';
}
