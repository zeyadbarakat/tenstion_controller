/**
 * @file pi_controller.c
 * @brief Reusable PI Controller Implementation
 *
 * PI Controller Theory:
 * ====================
 *
 * The PI controller computes:
 *   output = Kp * error + Ki * ∫error dt
 *
 * In discrete form (with sample time dt):
 *   output[n] = Kp * error[n] + Ki * Σ(error[i] * dt)
 *
 * Anti-Windup:
 * ===========
 * Integral windup occurs when the controller output is saturated but
 * the error persists. The integral term continues to grow (wind up),
 * causing slow response when the error finally reverses.
 *
 * Two anti-windup methods are implemented:
 *
 * 1. Integral Clamping:
 *    - Limit integral term to predefined bounds
 *    - Simple but may still allow some windup
 *
 * 2. Back-Calculation (conditional integration):
 *    - When output is saturated, don't accumulate integral
 *    - More sophisticated, prevents windup entirely
 *    - Used here: if output saturated, integral += 0
 *
 * This is critical in unwinding applications where:
 * - Tension may exceed setpoint during ramp-up
 * - Speed may hit max limit during disturbance
 * - Integral would otherwise wind up excessively
 *
 * Bumpless Transfer:
 * =================
 * When changing controller parameters or switching between manual/auto:
 * - Pre-load integral to produce current output at current error
 * - Prevents sudden output jumps (bumps)
 *
 * Tuning Guidelines:
 * =================
 * For cascaded control (inner speed loop, outer tension loop):
 *
 * Speed Loop (fast):
 * - Start with Kp = 0.5, Ki = 0.1
 * - Increase Kp until slight overshoot
 * - Increase Ki until setpoint reached quickly
 * - Typical: Kp = 0.3-1.0, Ki = 0.05-0.2
 *
 * Tension Loop (slow):
 * - Must be slower than speed loop (5-10x slower)
 * - Start with Kp = 0.2, Ki = 0.05
 * - Typical: Kp = 0.1-0.5, Ki = 0.02-0.1
 */

#include "pi_controller.h"
#include <math.h>
#include <string.h>


/*******************************************************************************
 * Private Definitions
 ******************************************************************************/

#define EPSILON 0.0001f // Float comparison epsilon

/*******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/

static float clamp(float value, float min, float max);

/*******************************************************************************
 * Public API Implementation
 ******************************************************************************/

void pi_init(pi_controller_t *ctrl, float kp, float ki, float dt, float out_min,
             float out_max) {
  if (ctrl == NULL) {
    return;
  }

  memset(ctrl, 0, sizeof(pi_controller_t));

  ctrl->kp = kp;
  ctrl->ki = ki;
  ctrl->dt = dt;
  ctrl->output_min = out_min;
  ctrl->output_max = out_max;

  // Set default integral limits based on output limits
  // This prevents integral from growing larger than needed
  float output_range = out_max - out_min;
  if (ki > EPSILON) {
    ctrl->integral_max = output_range / ki;
    ctrl->integral_min = -ctrl->integral_max;
  } else {
    ctrl->integral_max = output_range;
    ctrl->integral_min = -output_range;
  }

  ctrl->enabled = false; // Start disabled
  ctrl->first_run = true;
}

float pi_compute(pi_controller_t *ctrl, float setpoint, float measured_value) {
  if (ctrl == NULL) {
    return 0.0f;
  }

  if (!ctrl->enabled) {
    return 0.0f;
  }

  // Calculate error
  float error = setpoint - measured_value;
  ctrl->last_error = error;

  // Handle first run - don't apply full integral on first execution
  if (ctrl->first_run) {
    ctrl->prev_error = error;
    ctrl->prev_setpoint = setpoint;
    ctrl->first_run = false;
  }

  // ========== Proportional Term ==========
  float p_term = ctrl->kp * error;
  ctrl->last_p_term = p_term;

  // ========== Integral Term ==========
  // Use trapezoidal integration for better accuracy
  float error_avg = (error + ctrl->prev_error) / 2.0f;

  // Conditional integration (anti-windup):
  // Only integrate if:
  // 1. Output is not saturated, OR
  // 2. Error would drive integral toward zero
  bool integrate = true;

  if (ctrl->saturated) {
    // Check if error would help reduce saturation
    if ((ctrl->prev_output >= ctrl->output_max && error > 0) ||
        (ctrl->prev_output <= ctrl->output_min && error < 0)) {
      integrate = false; // Would make saturation worse
    }
  }

  if (integrate) {
    ctrl->integral += error_avg * ctrl->dt;

    // Clamp integral to prevent excessive accumulation
    ctrl->integral =
        clamp(ctrl->integral, ctrl->integral_min, ctrl->integral_max);
  }

  float i_term = ctrl->ki * ctrl->integral;
  ctrl->last_i_term = i_term;

  // ========== Calculate Output ==========
  float output = p_term + i_term;

  // Check if output would be saturated
  float output_unclamped = output;
  output = clamp(output, ctrl->output_min, ctrl->output_max);

  ctrl->saturated = (fabsf(output - output_unclamped) > EPSILON);

  // ========== Back-Calculation Anti-Windup ==========
  // If output was clamped, adjust integral to match actual output
  if (ctrl->saturated && fabsf(ctrl->ki) > EPSILON) {
    // Calculate what integral should be for actual output
    float desired_integral = (output - p_term) / ctrl->ki;

    // Blend toward desired value (gradual correction)
    float correction = (desired_integral - ctrl->integral) * 0.5f;
    ctrl->integral += correction;
  }

  // Save state for next iteration
  ctrl->prev_error = error;
  ctrl->prev_output = output;
  ctrl->prev_setpoint = setpoint;

  return output;
}

void pi_reset(pi_controller_t *ctrl) {
  if (ctrl == NULL) {
    return;
  }

  ctrl->integral = 0.0f;
  ctrl->prev_error = 0.0f;
  ctrl->prev_output = 0.0f;
  ctrl->prev_setpoint = 0.0f;
  ctrl->first_run = true;
  ctrl->saturated = false;
  ctrl->last_p_term = 0.0f;
  ctrl->last_i_term = 0.0f;
  ctrl->last_error = 0.0f;
}

void pi_set_gains(pi_controller_t *ctrl, float kp, float ki) {
  if (ctrl == NULL) {
    return;
  }

  // Bumpless transfer: adjust integral to maintain current output
  if (fabsf(ctrl->ki) > EPSILON && fabsf(ki) > EPSILON) {
    // Scale integral so that Ki * integral produces same value
    ctrl->integral = ctrl->integral * ctrl->ki / ki;
  }

  ctrl->kp = kp;
  ctrl->ki = ki;

  // Recalculate integral limits
  if (ki > EPSILON) {
    float output_range = ctrl->output_max - ctrl->output_min;
    ctrl->integral_max = output_range / ki;
    ctrl->integral_min = -ctrl->integral_max;
  }
}

void pi_set_limits(pi_controller_t *ctrl, float out_min, float out_max) {
  if (ctrl == NULL) {
    return;
  }

  ctrl->output_min = out_min;
  ctrl->output_max = out_max;

  // Update integral limits
  if (fabsf(ctrl->ki) > EPSILON) {
    float output_range = out_max - out_min;
    ctrl->integral_max = output_range / ctrl->ki;
    ctrl->integral_min = -ctrl->integral_max;
  }
}

void pi_set_integral_limits(pi_controller_t *ctrl, float int_min,
                            float int_max) {
  if (ctrl == NULL) {
    return;
  }

  ctrl->integral_min = int_min;
  ctrl->integral_max = int_max;

  // Clamp current integral if necessary
  ctrl->integral = clamp(ctrl->integral, int_min, int_max);
}

void pi_set_enabled(pi_controller_t *ctrl, bool enabled) {
  if (ctrl == NULL) {
    return;
  }

  if (!ctrl->enabled && enabled) {
    // Being enabled - set first_run for bumpless start
    ctrl->first_run = true;
  }

  ctrl->enabled = enabled;
}

bool pi_is_enabled(const pi_controller_t *ctrl) {
  return ctrl != NULL && ctrl->enabled;
}

bool pi_is_saturated(const pi_controller_t *ctrl) {
  return ctrl != NULL && ctrl->saturated;
}

void pi_get_terms(const pi_controller_t *ctrl, float *p, float *i) {
  if (ctrl == NULL) {
    if (p)
      *p = 0.0f;
    if (i)
      *i = 0.0f;
    return;
  }

  if (p)
    *p = ctrl->last_p_term;
  if (i)
    *i = ctrl->last_i_term;
}

float pi_get_error(const pi_controller_t *ctrl) {
  return (ctrl != NULL) ? ctrl->last_error : 0.0f;
}

void pi_set_sample_time(pi_controller_t *ctrl, float dt) {
  if (ctrl == NULL || dt <= 0) {
    return;
  }

  // Scale integral to maintain same effective Ki * integral
  ctrl->integral = ctrl->integral * ctrl->dt / dt;
  ctrl->dt = dt;
}

void pi_preset_integral(pi_controller_t *ctrl, float output, float error) {
  if (ctrl == NULL) {
    return;
  }

  // Calculate what integral should be to produce desired output
  // output = Kp * error + Ki * integral
  // integral = (output - Kp * error) / Ki

  float p_term = ctrl->kp * error;

  if (fabsf(ctrl->ki) > EPSILON) {
    ctrl->integral = (output - p_term) / ctrl->ki;
    ctrl->integral =
        clamp(ctrl->integral, ctrl->integral_min, ctrl->integral_max);
  }

  ctrl->prev_output = output;
  ctrl->prev_error = error;
  ctrl->first_run = false;
}

/*******************************************************************************
 * Private Function Implementations
 ******************************************************************************/

/**
 * @brief Clamp value to range
 */
static float clamp(float value, float min, float max) {
  if (value < min)
    return min;
  if (value > max)
    return max;
  return value;
}
