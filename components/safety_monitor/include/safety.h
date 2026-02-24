/**
 * @file safety.h
 * @brief Safety Monitor for Tension Control System
 *
 * Monitors and enforces safety limits with fault handling:
 * - Tension limits (max/min with hysteresis)
 * - Speed limits (max/min RPM)
 * - Motor stall detection
 * - Encoder failure detection
 * - Load cell disconnect detection
 * - Emergency stop handling
 * - Watchdog integration
 * - Fault logging to NVS
 */

#ifndef SAFETY_H
#define SAFETY_H

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
 * @brief System state
 */
typedef enum {
  SYSTEM_STATE_IDLE = 0,      /**< System ready, not running */
  SYSTEM_STATE_STARTING,      /**< Startup sequence in progress */
  SYSTEM_STATE_RUNNING,       /**< Normal operation */
  SYSTEM_STATE_STOPPING,      /**< Controlled stop in progress */
  SYSTEM_STATE_WARNING,       /**< Approaching limits */
  SYSTEM_STATE_FAULT,         /**< Fault detected, motor stopped */
  SYSTEM_STATE_EMERGENCY_STOP /**< E-stop activated */
} system_state_t;

/**
 * @brief Fault codes (bitmask)
 */
typedef enum {
  FAULT_NONE = 0x0000,
  FAULT_OVER_TENSION = 0x0001,
  FAULT_UNDER_TENSION = 0x0002,
  FAULT_OVER_SPEED = 0x0004,
  FAULT_MOTOR_STALL = 0x0008,
  FAULT_ENCODER_FAILURE = 0x0010,
  FAULT_LOADCELL_FAILURE = 0x0020,
  FAULT_EMERGENCY_STOP = 0x0040,
  FAULT_WATCHDOG_TIMEOUT = 0x0080,
  FAULT_CALIBRATION_INVALID = 0x0100,
  FAULT_SENSOR_DISCONNECT = 0x0200,
  FAULT_COMMUNICATION = 0x0400,
} fault_code_t;

/**
 * @brief Safety limits configuration
 */
typedef struct {
  float max_tension_kg;
  float min_tension_kg;
  float max_speed_rpm;
  float min_speed_rpm;
  float warning_threshold; /**< Fraction of limit to trigger warning (e.g., 0.9)
                            */
  uint32_t stall_timeout_ms;
  uint32_t encoder_timeout_ms;
} safety_limits_t;

/**
 * @brief Current sensor readings for safety check
 */
typedef struct {
  float tension_kg;
  float speed_rpm;
  float pwm_percent;
  bool encoder_active;
  bool loadcell_connected;
  bool estop_active;
} safety_readings_t;

/**
 * @brief Safety fault callback type
 */
typedef void (*safety_fault_callback_t)(fault_code_t fault, void *user_data);

/**
 * @brief Safety monitor handle
 */
typedef struct safety_s *safety_handle_t;

/*******************************************************************************
 * Public API Functions
 ******************************************************************************/

/**
 * @brief Initialize safety monitor
 *
 * @param[out] handle   Returns handle on success
 *
 * @return ESP_OK on success
 */
esp_err_t safety_init(safety_handle_t *handle);

/**
 * @brief Deinitialize safety monitor
 *
 * @param[in] handle    Safety handle
 */
void safety_deinit(safety_handle_t handle);

/**
 * @brief Check all safety conditions
 *
 * Call this every control loop iteration.
 *
 * @param[in] handle    Safety handle
 * @param[in] readings  Current sensor readings
 *
 * @return Current system state
 */
system_state_t safety_check(safety_handle_t handle,
                            const safety_readings_t *readings);

/**
 * @brief Set safety limits
 *
 * @param[in] handle    Safety handle
 * @param[in] limits    Safety limits configuration
 *
 * @return ESP_OK on success
 */
esp_err_t safety_set_limits(safety_handle_t handle,
                            const safety_limits_t *limits);

/**
 * @brief Get current safety limits
 *
 * @param[in] handle    Safety handle
 * @param[out] limits   Returns current limits
 *
 * @return ESP_OK on success
 */
esp_err_t safety_get_limits(safety_handle_t handle, safety_limits_t *limits);

/**
 * @brief Get current system state
 *
 * @param[in] handle    Safety handle
 *
 * @return Current system state
 */
system_state_t safety_get_state(safety_handle_t handle);

/**
 * @brief Get active fault codes
 *
 * @param[in] handle    Safety handle
 *
 * @return Bitmask of active faults
 */
fault_code_t safety_get_faults(safety_handle_t handle);

/**
 * @brief Get fault description string
 *
 * @param[in] fault     Fault code
 *
 * @return Human-readable fault description
 */
const char *safety_get_fault_string(fault_code_t fault);

/**
 * @brief Attempt to clear faults
 *
 * Only clears if fault condition no longer exists.
 *
 * @param[in] handle    Safety handle
 *
 * @return ESP_OK if faults cleared
 */
esp_err_t safety_reset_fault(safety_handle_t handle);

/**
 * @brief Trigger emergency stop
 *
 * @param[in] handle    Safety handle
 */
void safety_emergency_stop(safety_handle_t handle);

/**
 * @brief Check if safe to start system
 *
 * Checks all preconditions for safe startup.
 *
 * @param[in] handle    Safety handle
 *
 * @return true if safe to start
 */
bool safety_is_safe_to_start(safety_handle_t handle);

/**
 * @brief Request system start
 *
 * Changes state from IDLE to STARTING.
 *
 * @param[in] handle    Safety handle
 *
 * @return ESP_OK if start allowed
 */
esp_err_t safety_request_start(safety_handle_t handle);

/**
 * @brief Request controlled stop
 *
 * @param[in] handle    Safety handle
 */
void safety_request_stop(safety_handle_t handle);

/**
 * @brief Set fault callback
 *
 * @param[in] handle    Safety handle
 * @param[in] callback  Callback function
 * @param[in] user_data User data passed to callback
 */
void safety_set_fault_callback(safety_handle_t handle,
                               safety_fault_callback_t callback,
                               void *user_data);

/**
 * @brief Feed watchdog (indicate system is alive)
 *
 * @param[in] handle    Safety handle
 */
void safety_feed_watchdog(safety_handle_t handle);

/**
 * @brief Log fault to NVS
 *
 * @param[in] handle    Safety handle
 * @param[in] fault     Fault to log
 */
void safety_log_fault(safety_handle_t handle, fault_code_t fault);

/**
 * @brief Get fault log count
 *
 * @param[in] handle    Safety handle
 *
 * @return Number of logged faults
 */
uint32_t safety_get_fault_count(safety_handle_t handle);

/**
 * @brief Save limits to NVS
 *
 * @param[in] handle    Safety handle
 *
 * @return ESP_OK on success
 */
esp_err_t safety_save_limits(safety_handle_t handle);

/**
 * @brief Load limits from NVS
 *
 * @param[in] handle    Safety handle
 *
 * @return ESP_OK on success
 */
esp_err_t safety_load_limits(safety_handle_t handle);

/**
 * @brief Get default safety limits
 *
 * @return Default limits configuration
 */
safety_limits_t safety_get_default_limits(void);

/**
 * @brief Get state name string
 *
 * @param[in] state     System state
 *
 * @return State name string
 */
const char *safety_get_state_string(system_state_t state);

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_H */
