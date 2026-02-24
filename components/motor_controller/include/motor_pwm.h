/**
 * @file motor_pwm.h
 * @brief DC Motor Controller using ESP32-S3 LEDC PWM Peripheral
 *
 * This driver provides motor control features:
 * - 25kHz PWM for silent operation (above audible range)
 * - 10-bit resolution (0-1023) for smooth control
 * - Bidirectional control with direction GPIO
 * - Soft-start with configurable ramp rate
 * - Emergency stop functionality
 * - Deadband compensation
 */

#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

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
 * @brief Motor direction
 */
typedef enum {
  MOTOR_DIR_FORWARD = 1,  /**< Forward/CW direction */
  MOTOR_DIR_REVERSE = -1, /**< Reverse/CCW direction */
  MOTOR_DIR_STOPPED = 0   /**< Motor stopped */
} motor_direction_t;

/**
 * @brief Motor state
 */
typedef enum {
  MOTOR_STATE_IDLE,    /**< Motor idle (no PWM) */
  MOTOR_STATE_RUNNING, /**< Motor running normally */
  MOTOR_STATE_RAMPING, /**< Motor ramping up/down */
  MOTOR_STATE_BRAKING, /**< Motor braking */
  MOTOR_STATE_FAULT    /**< Motor in fault state */
} motor_state_t;

/**
 * @brief Motor configuration structure
 */
typedef struct {
  int gpio_pwm;           /**< GPIO for PWM output */
  int gpio_dir;           /**< GPIO for direction control */
  uint32_t pwm_freq_hz;   /**< PWM frequency (default 25000) */
  uint8_t pwm_resolution; /**< PWM resolution in bits (default 10) */
  float ramp_rate;        /**< Ramp rate in %/second (default 100) */
  float deadband_percent; /**< Deadband compensation (0-10%) */
} motor_config_t;

/**
 * @brief Motor handle (opaque pointer)
 */
typedef struct motor_s *motor_handle_t;

/*******************************************************************************
 * Public API Functions
 ******************************************************************************/

/**
 * @brief Initialize motor controller
 *
 * Configures LEDC peripheral and GPIOs for motor control.
 *
 * @param[in] config    Motor configuration
 * @param[out] handle   Returns motor handle on success
 *
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG: Invalid parameters
 *      - ESP_ERR_NO_MEM: Memory allocation failed
 */
esp_err_t motor_init(const motor_config_t *config, motor_handle_t *handle);

/**
 * @brief Deinitialize motor and release resources
 *
 * @param[in] handle    Motor handle
 *
 * @return ESP_OK on success
 */
esp_err_t motor_deinit(motor_handle_t handle);

/**
 * @brief Set motor speed
 *
 * Sets the motor speed as a percentage. Positive values are forward,
 * negative values are reverse.
 *
 * @param[in] handle            Motor handle
 * @param[in] speed_percent     Speed from -100% to +100%
 *
 * @return ESP_OK on success
 */
esp_err_t motor_set_speed(motor_handle_t handle, float speed_percent);

/**
 * @brief Set motor duty cycle directly
 *
 * Low-level function to set PWM duty cycle directly.
 *
 * @param[in] handle    Motor handle
 * @param[in] duty      Duty cycle (0-1023 for 10-bit)
 * @param[in] forward   true for forward, false for reverse
 *
 * @return ESP_OK on success
 */
esp_err_t motor_set_duty(motor_handle_t handle, uint32_t duty, bool forward);

/**
 * @brief Get current speed setting
 *
 * @param[in] handle    Motor handle
 *
 * @return Current speed in percent (-100 to +100)
 */
float motor_get_speed(motor_handle_t handle);

/**
 * @brief Get current duty cycle
 *
 * @param[in] handle    Motor handle
 *
 * @return Current duty cycle (0-1023)
 */
uint32_t motor_get_duty(motor_handle_t handle);

/**
 * @brief Get motor direction
 *
 * @param[in] handle    Motor handle
 *
 * @return Current direction
 */
motor_direction_t motor_get_direction(motor_handle_t handle);

/**
 * @brief Get motor state
 *
 * @param[in] handle    Motor handle
 *
 * @return Current motor state
 */
motor_state_t motor_get_state(motor_handle_t handle);

/**
 * @brief Activate motor braking
 *
 * Sets PWM to 0 and activates braking mode.
 *
 * @param[in] handle    Motor handle
 *
 * @return ESP_OK on success
 */
esp_err_t motor_brake(motor_handle_t handle);

/**
 * @brief Coast motor (freewheel)
 *
 * Disables PWM output and lets motor coast.
 *
 * @param[in] handle    Motor handle
 *
 * @return ESP_OK on success
 */
esp_err_t motor_coast(motor_handle_t handle);

/**
 * @brief Soft start to target speed
 *
 * Gradually ramps the motor to target speed over the specified time.
 * This function is non-blocking - use motor_update() to progress the ramp.
 *
 * @param[in] handle            Motor handle
 * @param[in] target_percent    Target speed (-100 to +100)
 * @param[in] ramp_time_ms      Time to reach target (ms)
 *
 * @return ESP_OK on success
 */
esp_err_t motor_soft_start(motor_handle_t handle, float target_percent,
                           uint32_t ramp_time_ms);

/**
 * @brief Emergency stop
 *
 * Immediately stops the motor without ramping.
 *
 * @param[in] handle    Motor handle
 *
 * @return ESP_OK on success
 */
esp_err_t motor_emergency_stop(motor_handle_t handle);

/**
 * @brief Update motor (call periodically for ramping)
 *
 * Should be called periodically (e.g., every 20ms) to update ramping.
 *
 * @param[in] handle    Motor handle
 *
 * @return ESP_OK on success
 */
esp_err_t motor_update(motor_handle_t handle);

/**
 * @brief Enable/disable motor output
 *
 * @param[in] handle    Motor handle
 * @param[in] enable    true to enable, false to disable
 *
 * @return ESP_OK on success
 */
esp_err_t motor_enable(motor_handle_t handle, bool enable);

/**
 * @brief Check if motor is enabled
 *
 * @param[in] handle    Motor handle
 *
 * @return true if enabled
 */
bool motor_is_enabled(motor_handle_t handle);

/**
 * @brief Set ramp rate
 *
 * @param[in] handle    Motor handle
 * @param[in] rate      Ramp rate in percent per second
 *
 * @return ESP_OK on success
 */
esp_err_t motor_set_ramp_rate(motor_handle_t handle, float rate);

/**
 * @brief Set deadband compensation
 *
 * Deadband compensation adds a minimum PWM value to overcome
 * motor/driver static friction.
 *
 * @param[in] handle            Motor handle
 * @param[in] deadband_percent  Deadband as percentage (0-10)
 *
 * @return ESP_OK on success
 */
esp_err_t motor_set_deadband(motor_handle_t handle, float deadband_percent);

/**
 * @brief Create default motor configuration
 *
 * @return Default configuration structure
 */
motor_config_t motor_get_default_config(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_PWM_H */
