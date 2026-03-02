/**
 * @file safety.c
 * @brief Safety Monitor Implementation
 */

#include "safety.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <math.h>
#include <string.h>

static const char *TAG = "safety";

/*******************************************************************************
 * Private Definitions
 ******************************************************************************/

#define NVS_NAMESPACE "safety"
#define HYSTERESIS_PERCENT 5.0f // 5% hysteresis on limits

/*******************************************************************************
 * Private Structures
 ******************************************************************************/

struct safety_s {
  // Configuration
  safety_limits_t limits;

  // State
  system_state_t state;
  fault_code_t active_faults;

  // Monitoring
  int64_t last_encoder_pulse_us;
  int64_t stall_start_us;
  int64_t watchdog_last_feed_us;
  bool in_stall_condition;

  // Fault tracking
  uint32_t fault_count;

  // Callback
  safety_fault_callback_t fault_callback;
  void *callback_user_data;

  // Thread safety
  SemaphoreHandle_t mutex;
};

/*******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/

static void set_fault(safety_handle_t handle, fault_code_t fault);
static void clear_fault(safety_handle_t handle, fault_code_t fault);
static bool check_limits_ok(safety_handle_t handle,
                            const safety_readings_t *readings);

/*******************************************************************************
 * Public API Implementation
 ******************************************************************************/

safety_limits_t safety_get_default_limits(void) {
  safety_limits_t limits = {
      .max_tension_kg = 10.0f,
      .min_tension_kg = 0.5f,
      .max_speed_rpm = 2000.0f,
      .min_speed_rpm = 0.0f,
      .warning_threshold = 0.9f,
      .stall_timeout_ms = 2000,
      .encoder_timeout_ms = 2000,
  };
  return limits;
}

esp_err_t safety_init(safety_handle_t *handle) {
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  struct safety_s *safety = calloc(1, sizeof(struct safety_s));
  if (safety == NULL) {
    return ESP_ERR_NO_MEM;
  }

  safety->limits = safety_get_default_limits();
  safety->state = SYSTEM_STATE_IDLE;
  safety->active_faults = FAULT_NONE;

  int64_t now = esp_timer_get_time();
  safety->last_encoder_pulse_us = now;
  safety->watchdog_last_feed_us = now;

  safety->mutex = xSemaphoreCreateMutex();
  if (safety->mutex == NULL) {
    free(safety);
    return ESP_ERR_NO_MEM;
  }

  ESP_LOGI(TAG, "Safety monitor initialized");

  *handle = safety;
  return ESP_OK;
}

void safety_deinit(safety_handle_t handle) {
  if (handle) {
    vSemaphoreDelete(handle->mutex);
    free(handle);
  }
}

system_state_t safety_check(safety_handle_t handle,
                            const safety_readings_t *readings) {
  if (handle == NULL || readings == NULL) {
    return SYSTEM_STATE_FAULT;
  }

  xSemaphoreTake(handle->mutex, portMAX_DELAY);

  int64_t now = esp_timer_get_time();
  system_state_t prev_state = handle->state;

  // === Emergency Stop Check (highest priority) ===
  if (readings->estop_active) {
    if (!(handle->active_faults & FAULT_EMERGENCY_STOP)) {
      set_fault(handle, FAULT_EMERGENCY_STOP);
      ESP_LOGW(TAG, "EMERGENCY STOP ACTIVATED");
    }
    handle->state = SYSTEM_STATE_EMERGENCY_STOP;
    xSemaphoreGive(handle->mutex);
    return handle->state;
  } else {
    clear_fault(handle, FAULT_EMERGENCY_STOP);
  }

  // === Sensor Connectivity Checks ===
  if (!readings->loadcell_connected) {
    set_fault(handle, FAULT_LOADCELL_FAILURE);
  } else {
    clear_fault(handle, FAULT_LOADCELL_FAILURE);
  }

  // Encoder failure: only check when RUNNING AND motor is actually driving
  // Must provide enough PWM to overcome static friction before expecting
  // movement
  if (!readings->encoder_active && handle->state == SYSTEM_STATE_RUNNING &&
      readings->pwm_percent > 15.0f) {
    int64_t idle_time_ms = (now - handle->last_encoder_pulse_us) / 1000;
    if (idle_time_ms > handle->limits.encoder_timeout_ms) {
      set_fault(handle, FAULT_ENCODER_FAILURE);
    }
  } else {
    handle->last_encoder_pulse_us = now;
    clear_fault(handle, FAULT_ENCODER_FAILURE);
  }

  // === Transition STARTING → RUNNING after successful safety check ===
  if (handle->state == SYSTEM_STATE_STARTING &&
      handle->active_faults == FAULT_NONE) {
    handle->state = SYSTEM_STATE_RUNNING;
    ESP_LOGI(TAG, "Safety checks passed - transitioning to RUNNING");
  }

  // === Only check operational limits when running ===
  if (handle->state == SYSTEM_STATE_RUNNING ||
      handle->state == SYSTEM_STATE_WARNING) {
    // Tension limits
    float max_with_hyst =
        handle->limits.max_tension_kg * (1.0f + HYSTERESIS_PERCENT / 100.0f);
    float min_with_hyst =
        handle->limits.min_tension_kg * (1.0f - HYSTERESIS_PERCENT / 100.0f);

    if (readings->tension_kg > max_with_hyst) {
      set_fault(handle, FAULT_OVER_TENSION);
    } else if (readings->tension_kg < handle->limits.max_tension_kg) {
      clear_fault(handle, FAULT_OVER_TENSION);
    }

    if (readings->tension_kg < min_with_hyst && readings->tension_kg > 0.01f) {
      set_fault(handle, FAULT_UNDER_TENSION);
    } else if (readings->tension_kg > handle->limits.min_tension_kg) {
      clear_fault(handle, FAULT_UNDER_TENSION);
    }

    // Speed limits
    if (fabsf(readings->speed_rpm) > handle->limits.max_speed_rpm) {
      set_fault(handle, FAULT_OVER_SPEED);
    } else {
      clear_fault(handle, FAULT_OVER_SPEED);
    }

    // Motor stall detection
    if (fabsf(readings->speed_rpm) < 10.0f && readings->pwm_percent > 30.0f) {
      if (!handle->in_stall_condition) {
        handle->stall_start_us = now;
        handle->in_stall_condition = true;
      } else if ((now - handle->stall_start_us) / 1000 >
                 handle->limits.stall_timeout_ms) {
        set_fault(handle, FAULT_MOTOR_STALL);
      }
    } else {
      handle->in_stall_condition = false;
      clear_fault(handle, FAULT_MOTOR_STALL);
    }

    // Check for warning conditions
    float tension_percent =
        readings->tension_kg / handle->limits.max_tension_kg;
    float speed_percent =
        fabsf(readings->speed_rpm) / handle->limits.max_speed_rpm;

    if (tension_percent > handle->limits.warning_threshold ||
        speed_percent > handle->limits.warning_threshold) {
      if (handle->active_faults == FAULT_NONE) {
        handle->state = SYSTEM_STATE_WARNING;
      }
    } else if (handle->state == SYSTEM_STATE_WARNING &&
               handle->active_faults == FAULT_NONE) {
      handle->state = SYSTEM_STATE_RUNNING;
    }
  }

  // === Set state based on faults ===
  if (handle->active_faults != FAULT_NONE &&
      handle->state != SYSTEM_STATE_EMERGENCY_STOP) {
    handle->state = SYSTEM_STATE_FAULT;
  } else if (handle->active_faults == FAULT_NONE &&
             (handle->state == SYSTEM_STATE_FAULT ||
              handle->state == SYSTEM_STATE_EMERGENCY_STOP)) {
    // Auto-recover: all faults cleared → return to IDLE
    handle->state = SYSTEM_STATE_IDLE;
    ESP_LOGI(TAG, "All faults cleared - returning to IDLE");
  }

  // Log state transitions
  if (handle->state != prev_state) {
    ESP_LOGI(TAG, "State changed: %s -> %s",
             safety_get_state_string(prev_state),
             safety_get_state_string(handle->state));
  }

  xSemaphoreGive(handle->mutex);
  return handle->state;
}

esp_err_t safety_set_limits(safety_handle_t handle,
                            const safety_limits_t *limits) {
  if (handle == NULL || limits == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  memcpy(&handle->limits, limits, sizeof(safety_limits_t));
  xSemaphoreGive(handle->mutex);

  ESP_LOGI(TAG,
           "Safety limits updated: tension=%.1f-%.1f kg, speed=%.0f-%.0f rpm",
           limits->min_tension_kg, limits->max_tension_kg,
           limits->min_speed_rpm, limits->max_speed_rpm);

  return ESP_OK;
}

esp_err_t safety_get_limits(safety_handle_t handle, safety_limits_t *limits) {
  if (handle == NULL || limits == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  memcpy(limits, &handle->limits, sizeof(safety_limits_t));
  xSemaphoreGive(handle->mutex);

  return ESP_OK;
}

system_state_t safety_get_state(safety_handle_t handle) {
  if (handle == NULL)
    return SYSTEM_STATE_FAULT;

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  system_state_t state = handle->state;
  xSemaphoreGive(handle->mutex);

  return state;
}

fault_code_t safety_get_faults(safety_handle_t handle) {
  if (handle == NULL)
    return FAULT_NONE;

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  fault_code_t faults = handle->active_faults;
  xSemaphoreGive(handle->mutex);

  return faults;
}

const char *safety_get_fault_string(fault_code_t fault) {
  switch (fault) {
  case FAULT_NONE:
    return "No Fault";
  case FAULT_OVER_TENSION:
    return "Over Tension";
  case FAULT_UNDER_TENSION:
    return "Under Tension";
  case FAULT_OVER_SPEED:
    return "Over Speed";
  case FAULT_MOTOR_STALL:
    return "Motor Stall";
  case FAULT_ENCODER_FAILURE:
    return "Encoder Fail";
  case FAULT_LOADCELL_FAILURE:
    return "LoadCell Fail";
  case FAULT_EMERGENCY_STOP:
    return "E-STOP";
  case FAULT_WATCHDOG_TIMEOUT:
    return "Watchdog";
  case FAULT_CALIBRATION_INVALID:
    return "Not Calibrated";
  case FAULT_SENSOR_DISCONNECT:
    return "Sensor Disconnect";
  case FAULT_COMMUNICATION:
    return "Comm Error";
  default:
    return "Unknown";
  }
}

const char *safety_get_state_string(system_state_t state) {
  switch (state) {
  case SYSTEM_STATE_IDLE:
    return "IDLE";
  case SYSTEM_STATE_STARTING:
    return "STARTING";
  case SYSTEM_STATE_RUNNING:
    return "RUNNING";
  case SYSTEM_STATE_STOPPING:
    return "STOPPING";
  case SYSTEM_STATE_WARNING:
    return "WARNING";
  case SYSTEM_STATE_FAULT:
    return "FAULT";
  case SYSTEM_STATE_EMERGENCY_STOP:
    return "E-STOP";
  default:
    return "UNKNOWN";
  }
}

esp_err_t safety_reset_fault(safety_handle_t handle) {
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  xSemaphoreTake(handle->mutex, portMAX_DELAY);

  // Can only reset from FAULT or EMERGENCY_STOP state
  if (handle->state != SYSTEM_STATE_FAULT &&
      handle->state != SYSTEM_STATE_EMERGENCY_STOP) {
    xSemaphoreGive(handle->mutex);
    return ESP_ERR_INVALID_STATE;
  }

  // Clear all faults (they will re-trigger if conditions still exist)
  handle->active_faults = FAULT_NONE;
  handle->state = SYSTEM_STATE_IDLE;
  handle->in_stall_condition = false;

  ESP_LOGI(TAG, "Faults reset - returning to IDLE");

  xSemaphoreGive(handle->mutex);
  return ESP_OK;
}

void safety_emergency_stop(safety_handle_t handle) {
  if (handle == NULL)
    return;

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  set_fault(handle, FAULT_EMERGENCY_STOP);
  handle->state = SYSTEM_STATE_EMERGENCY_STOP;
  xSemaphoreGive(handle->mutex);

  ESP_LOGW(TAG, "Software emergency stop triggered");
}

bool safety_is_safe_to_start(safety_handle_t handle) {
  if (handle == NULL)
    return false;

  xSemaphoreTake(handle->mutex, portMAX_DELAY);

  bool safe = (handle->state == SYSTEM_STATE_IDLE) &&
              (handle->active_faults == FAULT_NONE);

  xSemaphoreGive(handle->mutex);
  return safe;
}

esp_err_t safety_request_start(safety_handle_t handle) {
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (!safety_is_safe_to_start(handle)) {
    return ESP_ERR_INVALID_STATE;
  }

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  handle->state = SYSTEM_STATE_STARTING;
  xSemaphoreGive(handle->mutex);

  ESP_LOGI(TAG, "Start requested - entering STARTING state");
  return ESP_OK;
}

void safety_request_stop(safety_handle_t handle) {
  if (handle == NULL)
    return;

  xSemaphoreTake(handle->mutex, portMAX_DELAY);

  if (handle->state == SYSTEM_STATE_RUNNING ||
      handle->state == SYSTEM_STATE_WARNING) {
    handle->state = SYSTEM_STATE_STOPPING;
    ESP_LOGI(TAG, "Stop requested - entering STOPPING state");
  }

  xSemaphoreGive(handle->mutex);
}

void safety_set_fault_callback(safety_handle_t handle,
                               safety_fault_callback_t callback,
                               void *user_data) {
  if (handle == NULL)
    return;

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  handle->fault_callback = callback;
  handle->callback_user_data = user_data;
  xSemaphoreGive(handle->mutex);
}

void safety_feed_watchdog(safety_handle_t handle) {
  if (handle == NULL)
    return;
  handle->watchdog_last_feed_us = esp_timer_get_time();
}

void safety_log_fault(safety_handle_t handle, fault_code_t fault) {
  if (handle == NULL)
    return;

  handle->fault_count++;
  ESP_LOGW(TAG, "Fault logged (#%lu): %s", (unsigned long)handle->fault_count,
           safety_get_fault_string(fault));
}

uint32_t safety_get_fault_count(safety_handle_t handle) {
  return (handle != NULL) ? handle->fault_count : 0;
}

esp_err_t safety_save_limits(safety_handle_t handle) {
  if (handle == NULL)
    return ESP_ERR_INVALID_ARG;

  nvs_handle_t nvs;
  esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
  if (ret != ESP_OK)
    return ret;

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  ret = nvs_set_blob(nvs, "limits", &handle->limits, sizeof(safety_limits_t));
  xSemaphoreGive(handle->mutex);

  if (ret == ESP_OK) {
    ret = nvs_commit(nvs);
  }
  nvs_close(nvs);

  return ret;
}

esp_err_t safety_load_limits(safety_handle_t handle) {
  if (handle == NULL)
    return ESP_ERR_INVALID_ARG;

  nvs_handle_t nvs;
  esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs);
  if (ret != ESP_OK)
    return ret;

  safety_limits_t limits;
  size_t size = sizeof(safety_limits_t);
  ret = nvs_get_blob(nvs, "limits", &limits, &size);

  if (ret == ESP_OK) {
    xSemaphoreTake(handle->mutex, portMAX_DELAY);
    memcpy(&handle->limits, &limits, sizeof(safety_limits_t));
    xSemaphoreGive(handle->mutex);

    ESP_LOGI(TAG, "Safety limits loaded from NVS");
  }

  nvs_close(nvs);
  return ret;
}

/*******************************************************************************
 * Private Function Implementations
 ******************************************************************************/

static void set_fault(safety_handle_t handle, fault_code_t fault) {
  if (!(handle->active_faults & fault)) {
    handle->active_faults |= fault;
    safety_log_fault(handle, fault);

    ESP_LOGW(TAG, "FAULT SET: %s", safety_get_fault_string(fault));

    if (handle->fault_callback) {
      handle->fault_callback(fault, handle->callback_user_data);
    }
  }
}

static void clear_fault(safety_handle_t handle, fault_code_t fault) {
  if (handle->active_faults & fault) {
    handle->active_faults &= ~fault;
    ESP_LOGI(TAG, "Fault cleared: %s", safety_get_fault_string(fault));
  }
}
