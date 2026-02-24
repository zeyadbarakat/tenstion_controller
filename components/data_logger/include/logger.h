/**
 * @file logger.h
 * @brief Data Logger for Tension Control System
 */

#ifndef LOGGER_H
#define LOGGER_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/

/**
 * @brief Log sample structure
 */
typedef struct {
  uint32_t timestamp_ms;
  float tension_setpoint;
  float tension_actual;
  float speed_setpoint;
  float speed_actual;
  float pwm_output;
  float tension_error;
  float speed_error;
  uint8_t state;
  uint8_t fault_flags;
} log_sample_t;

/**
 * @brief Performance metrics
 */
typedef struct {
  float settling_time_s;    /**< Time to reach ±2% of setpoint */
  float rise_time_s;        /**< Time from 10% to 90% of step */
  float overshoot_percent;  /**< Peak overshoot percentage */
  float steady_state_error; /**< Average error after settling */
  float iae;                /**< Integral Absolute Error */
  float ise;                /**< Integral Square Error */
  bool valid;               /**< Metrics are valid */
} performance_metrics_t;

/**
 * @brief Logger handle
 */
typedef struct logger_s *logger_handle_t;

/*******************************************************************************
 * Public API Functions
 ******************************************************************************/

esp_err_t logger_init(logger_handle_t *handle, uint16_t buffer_size);
void logger_deinit(logger_handle_t handle);
esp_err_t logger_log_sample(logger_handle_t handle, const log_sample_t *sample);
void logger_print_realtime(logger_handle_t handle);
void logger_print_latest(logger_handle_t handle, uint16_t num_samples);
esp_err_t logger_get_metrics(logger_handle_t handle,
                             performance_metrics_t *metrics);
void logger_clear_buffer(logger_handle_t handle);
uint16_t logger_get_count(logger_handle_t handle);
bool logger_get_sample(logger_handle_t handle, uint16_t index,
                       log_sample_t *sample);
void logger_enable(logger_handle_t handle, bool enable);
bool logger_is_enabled(logger_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* LOGGER_H */
