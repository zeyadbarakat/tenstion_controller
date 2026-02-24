/**
 * @file pi_controller.h
 * @brief Reusable PI Controller with Anti-Windup
 *
 * This controller implements:
 * - Proportional-Integral control
 * - Anti-windup with integral clamping and back-calculation
 * - Output saturation
 * - Bumpless transfer for parameter changes
 * - Sample time compensation
 *
 * Why PI (not PID) for this application:
 * - Derivative term amplifies encoder/sensor noise
 * - Speed and tension dynamics are first-order dominant
 * - Easier to tune (2 parameters vs 3)
 * - Industry standard for unwinding/tension control
 */

#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/

/**
 * @brief PI Controller structure
 *
 * All state is contained within this structure, allowing multiple
 * independent controllers to be instantiated.
 */
typedef struct {
  // Tuning parameters
  float kp; /**< Proportional gain */
  float ki; /**< Integral gain */
  float dt; /**< Sample time (seconds) */

  // State variables
  float integral;      /**< Integral accumulator */
  float prev_error;    /**< Previous error (for derivative if needed) */
  float prev_output;   /**< Previous output (for bumpless transfer) */
  float prev_setpoint; /**< Previous setpoint (for step detection) */

  // Output limits
  float output_min; /**< Minimum output limit */
  float output_max; /**< Maximum output limit */

  // Anti-windup limits
  float integral_min; /**< Minimum integral value */
  float integral_max; /**< Maximum integral value */

  // Status flags
  bool enabled;   /**< Controller enabled flag */
  bool first_run; /**< First execution flag */
  bool saturated; /**< Output is saturated */

  // Debug/monitoring values
  float last_p_term; /**< Last proportional term */
  float last_i_term; /**< Last integral term */
  float last_error;  /**< Last error value */
} pi_controller_t;

/*******************************************************************************
 * Public API Functions
 ******************************************************************************/

/**
 * @brief Initialize PI controller
 *
 * Sets up controller with specified gains and output limits.
 * Integral limits are automatically set based on output limits.
 *
 * @param[out] ctrl     Pointer to controller structure
 * @param[in] kp        Proportional gain
 * @param[in] ki        Integral gain
 * @param[in] dt        Sample time in seconds
 * @param[in] out_min   Minimum output value
 * @param[in] out_max   Maximum output value
 */
void pi_init(pi_controller_t *ctrl, float kp, float ki, float dt, float out_min,
             float out_max);

/**
 * @brief Compute PI controller output
 *
 * Calculates the control output based on setpoint and measured value.
 *
 * Algorithm:
 *   error = setpoint - measured
 *   P = Kp * error
 *   integral += error * dt
 *   I = Ki * integral
 *   output = P + I (clamped to limits)
 *   Apply anti-windup if saturated
 *
 * @param[in,out] ctrl          Controller structure
 * @param[in] setpoint          Desired value
 * @param[in] measured_value    Current measured value
 *
 * @return Control output (clamped to output limits)
 */
float pi_compute(pi_controller_t *ctrl, float setpoint, float measured_value);

/**
 * @brief Reset controller state
 *
 * Clears integral and resets all state variables.
 * Call when stopping/restarting control.
 *
 * @param[in,out] ctrl  Controller structure
 */
void pi_reset(pi_controller_t *ctrl);

/**
 * @brief Set controller gains
 *
 * Updates Kp and Ki gains with bumpless transfer.
 *
 * @param[in,out] ctrl  Controller structure
 * @param[in] kp        New proportional gain
 * @param[in] ki        New integral gain
 */
void pi_set_gains(pi_controller_t *ctrl, float kp, float ki);

/**
 * @brief Set output limits
 *
 * Updates output saturation limits.
 * Also updates integral limits accordingly.
 *
 * @param[in,out] ctrl      Controller structure
 * @param[in] out_min       Minimum output
 * @param[in] out_max       Maximum output
 */
void pi_set_limits(pi_controller_t *ctrl, float out_min, float out_max);

/**
 * @brief Set integral limits for anti-windup
 *
 * @param[in,out] ctrl      Controller structure
 * @param[in] int_min       Minimum integral value
 * @param[in] int_max       Maximum integral value
 */
void pi_set_integral_limits(pi_controller_t *ctrl, float int_min,
                            float int_max);

/**
 * @brief Enable/disable controller
 *
 * When disabled, output is zero and integral is frozen.
 * When re-enabled, uses previous integral for bumpless transfer.
 *
 * @param[in,out] ctrl  Controller structure
 * @param[in] enabled   true to enable, false to disable
 */
void pi_set_enabled(pi_controller_t *ctrl, bool enabled);

/**
 * @brief Check if controller is enabled
 *
 * @param[in] ctrl  Controller structure
 *
 * @return true if enabled
 */
bool pi_is_enabled(const pi_controller_t *ctrl);

/**
 * @brief Check if output is saturated
 *
 * @param[in] ctrl  Controller structure
 *
 * @return true if last output was clamped
 */
bool pi_is_saturated(const pi_controller_t *ctrl);

/**
 * @brief Get individual control terms
 *
 * Returns the last calculated P and I terms for debugging/tuning.
 *
 * @param[in] ctrl  Controller structure
 * @param[out] p    Proportional term (can be NULL)
 * @param[out] i    Integral term (can be NULL)
 */
void pi_get_terms(const pi_controller_t *ctrl, float *p, float *i);

/**
 * @brief Get last error value
 *
 * @param[in] ctrl  Controller structure
 *
 * @return Last error (setpoint - measured)
 */
float pi_get_error(const pi_controller_t *ctrl);

/**
 * @brief Set sample time
 *
 * @param[in,out] ctrl  Controller structure
 * @param[in] dt        Sample time in seconds
 */
void pi_set_sample_time(pi_controller_t *ctrl, float dt);

/**
 * @brief Pre-load integral for bumpless start
 *
 * Sets the integral term to produce a specific output at the
 * current error. Used for bumpless transfer from manual to auto.
 *
 * @param[in,out] ctrl      Controller structure
 * @param[in] output        Desired initial output
 * @param[in] error         Current error value
 */
void pi_preset_integral(pi_controller_t *ctrl, float output, float error);

#ifdef __cplusplus
}
#endif

#endif /* PI_CONTROLLER_H */
