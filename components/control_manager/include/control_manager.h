/**
 * @file control_manager.h
 * @brief Central Control Manager for Tension Control System
 *
 * Orchestrates all components:
 * - Hardware drivers (encoder, load cell, motor)
 * - Control algorithms (PI controllers, auto-tuner)
 * - Safety monitoring
 * - User interfaces (buttons, LCD)
 * - Data logging
 */

#ifndef CONTROL_MANAGER_H
#define CONTROL_MANAGER_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

// Forward declare handles
typedef struct control_manager_s *control_manager_handle_t;

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/

/**
 * @brief Operating mode
 */
typedef enum {
  MODE_IDLE = 0,
  MODE_MANUAL,         /**< Manual speed control */
  MODE_AUTO_TENSION,   /**< Automatic tension control */
  MODE_TUNING_SPEED,   /**< Auto-tuning speed loop */
  MODE_TUNING_TENSION, /**< Auto-tuning tension loop */
  MODE_CALIBRATING     /**< Load cell calibration */
} control_mode_t;

/**
 * @brief System status (read by UI)
 */
typedef struct {
  // Measurements
  float tension_kg;
  float speed_rpm;
  float pwm_percent;

  // Setpoints
  float tension_setpoint;
  float speed_setpoint;

  // Control info
  float tension_error;
  float speed_error;

  // State
  control_mode_t mode;
  uint8_t system_state;
  uint16_t fault_flags;

  // Status
  bool calibrated;
  bool tuned;
  uint32_t uptime_seconds;
} system_status_t;

/**
 * @brief Control manager configuration
 */
typedef struct {
  // GPIO pins
  int encoder_a_gpio;
  int encoder_b_gpio;
  int hx711_data_gpio;
  int hx711_clock_gpio;
  int motor_pwm_gpio;
  int motor_dir_gpio;
  int button_run_gpio;
  int button_stop_gpio;
  int button_estop_gpio;

  // Timing
  uint32_t control_period_ms; /**< Control loop period */
  uint32_t ui_period_ms;      /**< UI update period */

  // Control parameters
  float default_tension_setpoint;
} control_manager_config_t;

/*******************************************************************************
 * Public API Functions
 ******************************************************************************/

/**
 * @brief Initialize the control manager
 *
 * Creates and initializes all components.
 *
 * @param[out] handle   Returns handle on success
 * @param[in] config    Configuration parameters
 *
 * @return ESP_OK on success
 */
esp_err_t control_manager_init(control_manager_handle_t *handle,
                               const control_manager_config_t *config);

/**
 * @brief Start control system
 *
 * Starts all FreeRTOS tasks.
 *
 * @param[in] handle    Control manager handle
 *
 * @return ESP_OK on success
 */
esp_err_t control_manager_start(control_manager_handle_t handle);

/**
 * @brief Stop control system
 *
 * @param[in] handle    Control manager handle
 */
void control_manager_stop(control_manager_handle_t handle);

/**
 * @brief Deinitialize control manager
 *
 * @param[in] handle    Control manager handle
 */
void control_manager_deinit(control_manager_handle_t handle);

/**
 * @brief Get current system status
 *
 * @param[in] handle    Control manager handle
 * @param[out] status   Returns current status
 */
void control_manager_get_status(control_manager_handle_t handle,
                                system_status_t *status);

/**
 * @brief Set tension setpoint
 *
 * @param[in] handle    Control manager handle
 * @param[in] tension   Tension setpoint in kg
 *
 * @return ESP_OK on success
 */
esp_err_t control_manager_set_tension(control_manager_handle_t handle,
                                      float tension);

/**
 * @brief Set operating mode
 *
 * @param[in] handle    Control manager handle
 * @param[in] mode      New operating mode
 *
 * @return ESP_OK on success
 */
esp_err_t control_manager_set_mode(control_manager_handle_t handle,
                                   control_mode_t mode);

/**
 * @brief Request system start (RUN command)
 *
 * @param[in] handle    Control manager handle
 *
 * @return ESP_OK if start successful
 */
esp_err_t control_manager_run(control_manager_handle_t handle);

/**
 * @brief Request controlled stop
 *
 * @param[in] handle    Control manager handle
 */
void control_manager_stop_controlled(control_manager_handle_t handle);

/**
 * @brief Trigger emergency stop
 *
 * @param[in] handle    Control manager handle
 */
void control_manager_emergency_stop(control_manager_handle_t handle);

/**
 * @brief Reset faults
 *
 * @param[in] handle    Control manager handle
 *
 * @return ESP_OK if faults cleared
 */
esp_err_t control_manager_reset_faults(control_manager_handle_t handle);

/**
 * @brief Start load cell tare calibration
 *
 * @param[in] handle    Control manager handle
 */
void control_manager_start_tare(control_manager_handle_t handle);

/**
 * @brief Start load cell span calibration
 *
 * @param[in] handle        Control manager handle
 * @param[in] known_weight  Known weight in kg
 */
void control_manager_start_calibration(control_manager_handle_t handle,
                                       float known_weight);

/**
 * @brief Start auto-tuning
 *
 * @param[in] handle    Control manager handle
 * @param[in] speed_loop    true for speed loop, false for tension loop
 */
void control_manager_start_autotune(control_manager_handle_t handle,
                                    bool speed_loop);

/**
 * @brief Manual jog motor
 *
 * Only works when system is in IDLE state.
 * Used for testing and setup.
 *
 * @param[in] handle    Control manager handle
 * @param[in] speed_percent  Motor speed (-100 to +100), 0 to stop
 */
void control_manager_jog(control_manager_handle_t handle, float speed_percent);

/**
 * @brief Get default configuration
 *
 * @return Default configuration
 */
control_manager_config_t control_manager_get_default_config(void);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_MANAGER_H */
