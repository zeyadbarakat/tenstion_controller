/**
 * @file logger.c
 * @brief Data Logger Implementation
 */

#include "logger.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>


static const char *TAG = "logger";

struct logger_s {
  log_sample_t *buffer;
  uint16_t buffer_size;
  uint16_t head;
  uint16_t count;
  bool enabled;
  log_sample_t latest;

  // Step response tracking
  bool tracking_step;
  float step_initial;
  float step_target;
  int64_t step_start_us;
  float step_peak;
  float iae_accumulator;
  float ise_accumulator;
  performance_metrics_t metrics;

  SemaphoreHandle_t mutex;
};

esp_err_t logger_init(logger_handle_t *handle, uint16_t buffer_size) {
  if (handle == NULL || buffer_size == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  struct logger_s *logger = calloc(1, sizeof(struct logger_s));
  if (logger == NULL) {
    return ESP_ERR_NO_MEM;
  }

  logger->buffer = calloc(buffer_size, sizeof(log_sample_t));
  if (logger->buffer == NULL) {
    free(logger);
    return ESP_ERR_NO_MEM;
  }

  logger->buffer_size = buffer_size;
  logger->mutex = xSemaphoreCreateMutex();
  if (logger->mutex == NULL) {
    free(logger->buffer);
    free(logger);
    return ESP_ERR_NO_MEM;
  }

  logger->enabled = true;

  ESP_LOGI(TAG, "Logger initialized: buffer=%u samples", buffer_size);

  *handle = logger;
  return ESP_OK;
}

void logger_deinit(logger_handle_t handle) {
  if (handle) {
    vSemaphoreDelete(handle->mutex);
    free(handle->buffer);
    free(handle);
  }
}

esp_err_t logger_log_sample(logger_handle_t handle,
                            const log_sample_t *sample) {
  if (handle == NULL || sample == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (!handle->enabled) {
    return ESP_OK;
  }

  xSemaphoreTake(handle->mutex, portMAX_DELAY);

  // Store in circular buffer
  memcpy(&handle->buffer[handle->head], sample, sizeof(log_sample_t));
  handle->head = (handle->head + 1) % handle->buffer_size;
  if (handle->count < handle->buffer_size) {
    handle->count++;
  }

  // Store latest for quick access
  memcpy(&handle->latest, sample, sizeof(log_sample_t));

  // Track step response metrics
  float error = fabsf(sample->tension_error);
  handle->iae_accumulator += error * 0.1f; // Assume 10Hz
  handle->ise_accumulator += error * error * 0.1f;

  // Detect step change
  if (!handle->tracking_step &&
      fabsf(sample->tension_setpoint - handle->step_target) > 0.5f) {
    // New step detected
    handle->tracking_step = true;
    handle->step_initial = sample->tension_actual;
    handle->step_target = sample->tension_setpoint;
    handle->step_start_us = esp_timer_get_time();
    handle->step_peak = sample->tension_actual;
    handle->iae_accumulator = 0;
    handle->ise_accumulator = 0;
    handle->metrics.valid = false;
  }

  if (handle->tracking_step) {
    // Track peak for overshoot
    if (sample->tension_actual > handle->step_peak) {
      handle->step_peak = sample->tension_actual;
    }

    // Check if settled (within 2% of setpoint)
    float settle_band = fabsf(sample->tension_setpoint) * 0.02f;
    if (settle_band < 0.01f)
      settle_band = 0.01f;

    if (fabsf(sample->tension_actual - sample->tension_setpoint) <
        settle_band) {
      float elapsed =
          (float)(esp_timer_get_time() - handle->step_start_us) / 1000000.0f;

      handle->metrics.settling_time_s = elapsed;
      handle->metrics.overshoot_percent =
          (handle->step_peak - handle->step_target) /
          (handle->step_target - handle->step_initial) * 100.0f;
      handle->metrics.steady_state_error = sample->tension_error;
      handle->metrics.iae = handle->iae_accumulator;
      handle->metrics.ise = handle->ise_accumulator;
      handle->metrics.valid = true;
      handle->tracking_step = false;
    }
  }

  xSemaphoreGive(handle->mutex);
  return ESP_OK;
}

void logger_print_realtime(logger_handle_t handle) {
  if (handle == NULL)
    return;

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  printf("T: %.2fkg (%.2f) | S: %.0frpm (%.0f) | PWM: %.1f%% | E: %.2f\n",
         handle->latest.tension_actual, handle->latest.tension_setpoint,
         handle->latest.speed_actual, handle->latest.speed_setpoint,
         handle->latest.pwm_output, handle->latest.tension_error);
  xSemaphoreGive(handle->mutex);
}

void logger_print_latest(logger_handle_t handle, uint16_t num_samples) {
  if (handle == NULL)
    return;

  xSemaphoreTake(handle->mutex, portMAX_DELAY);

  printf("time_ms,tension_sp,tension_pv,speed_sp,speed_pv,pwm,state\n");

  uint16_t count = (num_samples < handle->count) ? num_samples : handle->count;
  uint16_t start =
      (handle->head + handle->buffer_size - count) % handle->buffer_size;

  for (uint16_t i = 0; i < count; i++) {
    uint16_t idx = (start + i) % handle->buffer_size;
    log_sample_t *s = &handle->buffer[idx];
    printf("%lu,%.3f,%.3f,%.1f,%.1f,%.1f,%u\n", (unsigned long)s->timestamp_ms,
           s->tension_setpoint, s->tension_actual, s->speed_setpoint,
           s->speed_actual, s->pwm_output, s->state);
  }

  xSemaphoreGive(handle->mutex);
}

esp_err_t logger_get_metrics(logger_handle_t handle,
                             performance_metrics_t *metrics) {
  if (handle == NULL || metrics == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  memcpy(metrics, &handle->metrics, sizeof(performance_metrics_t));
  xSemaphoreGive(handle->mutex);

  return ESP_OK;
}

void logger_clear_buffer(logger_handle_t handle) {
  if (handle == NULL)
    return;

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  handle->head = 0;
  handle->count = 0;
  handle->iae_accumulator = 0;
  handle->ise_accumulator = 0;
  handle->tracking_step = false;
  memset(&handle->metrics, 0, sizeof(performance_metrics_t));
  xSemaphoreGive(handle->mutex);
}

uint16_t logger_get_count(logger_handle_t handle) {
  if (handle == NULL)
    return 0;
  return handle->count;
}

bool logger_get_sample(logger_handle_t handle, uint16_t index,
                       log_sample_t *sample) {
  if (handle == NULL || sample == NULL || index >= handle->count) {
    return false;
  }

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  uint16_t actual_idx =
      (handle->head + handle->buffer_size - handle->count + index) %
      handle->buffer_size;
  memcpy(sample, &handle->buffer[actual_idx], sizeof(log_sample_t));
  xSemaphoreGive(handle->mutex);

  return true;
}

void logger_enable(logger_handle_t handle, bool enable) {
  if (handle)
    handle->enabled = enable;
}

bool logger_is_enabled(logger_handle_t handle) {
  return (handle != NULL) && handle->enabled;
}
