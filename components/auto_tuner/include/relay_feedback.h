/**
 * @file relay_feedback.h
 * @brief Relay Feedback Auto-Tuning (Åström-Hägglund Method)
 *
 * This module implements automatic PI tuning using the relay feedback method:
 * 1. Apply relay (on-off) control instead of PI controller
 * 2. System oscillates at ultimate frequency
 * 3. Measure oscillation period (Tu) and amplitude (a)
 * 4. Calculate ultimate gain Ku = 4*d / (π*a)
 * 5. Apply Ziegler-Nichols tuning rules
 *
 * This method is well-suited for industrial control systems because:
 * - Works under closed-loop conditions (safe)
 * - Automatically determines system dynamics
 * - Produces conservative, stable tuning
 */

#ifndef RELAY_FEEDBACK_H
#define RELAY_FEEDBACK_H

#include "esp_err.h"
#include "pi_controller.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/

/**
 * @brief Auto-tune target loop
 */
typedef enum {
  AUTOTUNE_SPEED_LOOP = 0,   /**< Speed control loop */
  AUTOTUNE_TENSION_LOOP = 1, /**< Tension control loop */
} autotune_loop_t;

/**
 * @brief Auto-tune state
 */
typedef enum {
  TUNE_IDLE = 0,       /**< Not running */
  TUNE_INIT,           /**< Initializing */
  TUNE_WAITING_STABLE, /**< Waiting for stable starting point */
  TUNE_RELAY,          /**< Relay control active, measuring */
  TUNE_ANALYZING,      /**< Analyzing oscillation data */
  TUNE_COMPLETE,       /**< Tuning complete, results available */
  TUNE_FAILED,         /**< Tuning failed */
  TUNE_ABORTED         /**< Tuning aborted by user */
} autotune_state_t;

/**
 * @brief Tuning rules to apply
 */
typedef enum {
  TUNE_RULE_ZIEGLER_NICHOLS, /**< Classic ZN: Kp=0.45*Ku, Ki=1.2*Kp/Tu */
  TUNE_RULE_TYREUS_LUYBEN,   /**< More conservative: Kp=0.31*Ku */
  TUNE_RULE_SOME_OVERSHOOT,  /**< Allow ~10% overshoot */
  TUNE_RULE_NO_OVERSHOOT,    /**< Minimize overshoot */
} autotune_rule_t;

/**
 * @brief Auto-tune configuration
 */
typedef struct {
  autotune_loop_t loop;  /**< Which loop to tune */
  float relay_amplitude; /**< Relay output swing (e.g., 20 for ±20%) */
  float bias;            /**< Relay bias/center point (e.g., 50%) */
  float setpoint;        /**< Setpoint during tuning */
  uint8_t min_cycles;    /**< Minimum oscillation cycles (3-5) */
  uint32_t timeout_s;    /**< Maximum tuning time */
  autotune_rule_t rule;  /**< Tuning rule to apply */
} autotune_config_t;

/**
 * @brief Auto-tune results
 */
typedef struct {
  float ku;                /**< Ultimate gain */
  float tu;                /**< Ultimate period (seconds) */
  float kp;                /**< Calculated Kp */
  float ki;                /**< Calculated Ki */
  float oscillation_amp;   /**< Measured oscillation amplitude */
  uint8_t cycles_detected; /**< Number of cycles detected */
  bool valid;              /**< Results are valid */
} autotune_result_t;

/**
 * @brief Auto-tune status for UI display
 */
typedef struct {
  autotune_state_t state;
  uint8_t progress_percent;
  char message[64];
  float current_pv;     /**< Current process variable */
  float current_output; /**< Current relay output */
} autotune_status_t;

/**
 * @brief Auto-tuner handle
 */
typedef struct autotune_s *autotune_handle_t;

/*******************************************************************************
 * Public API Functions
 ******************************************************************************/

/**
 * @brief Initialize auto-tuner
 *
 * @param[out] handle   Returns handle on success
 *
 * @return ESP_OK on success
 */
esp_err_t autotune_init(autotune_handle_t *handle);

/**
 * @brief Deinitialize auto-tuner
 *
 * @param[in] handle    Auto-tuner handle
 */
void autotune_deinit(autotune_handle_t handle);

/**
 * @brief Start auto-tuning process
 *
 * @param[in] handle    Auto-tuner handle
 * @param[in] config    Tuning configuration
 *
 * @return ESP_OK on success
 */
esp_err_t autotune_start(autotune_handle_t handle,
                         const autotune_config_t *config);

/**
 * @brief Update auto-tuner (call every control cycle)
 *
 * This function implements the relay feedback algorithm.
 * Call it instead of the normal PI controller during tuning.
 *
 * @param[in] handle            Auto-tuner handle
 * @param[in] setpoint          Target setpoint
 * @param[in] measured_value    Current measured value
 *
 * @return Control output (relay output during tuning, or 0 if not tuning)
 */
float autotune_update(autotune_handle_t handle, float setpoint,
                      float measured_value);

/**
 * @brief Check if tuning is complete
 *
 * @param[in] handle    Auto-tuner handle
 *
 * @return true if tuning is complete (success or failure)
 */
bool autotune_is_complete(autotune_handle_t handle);

/**
 * @brief Check if tuning is active
 *
 * @param[in] handle    Auto-tuner handle
 *
 * @return true if tuning is in progress
 */
bool autotune_is_active(autotune_handle_t handle);

/**
 * @brief Get tuning results
 *
 * @param[in] handle    Auto-tuner handle
 * @param[out] result   Returns tuning results
 *
 * @return ESP_OK if results available
 */
esp_err_t autotune_get_result(autotune_handle_t handle,
                              autotune_result_t *result);

/**
 * @brief Get current status (for UI)
 *
 * @param[in] handle    Auto-tuner handle
 * @param[out] status   Returns current status
 */
void autotune_get_status(autotune_handle_t handle, autotune_status_t *status);

/**
 * @brief Apply results to PI controller
 *
 * @param[in] handle    Auto-tuner handle
 * @param[out] ctrl     PI controller to configure
 *
 * @return ESP_OK on success
 */
esp_err_t autotune_apply_result(autotune_handle_t handle,
                                pi_controller_t *ctrl);

/**
 * @brief Abort tuning
 *
 * @param[in] handle    Auto-tuner handle
 */
void autotune_abort(autotune_handle_t handle);

/**
 * @brief Save tuning results to NVS
 *
 * @param[in] handle    Auto-tuner handle
 * @param[in] loop      Which loop to save for
 *
 * @return ESP_OK on success
 */
esp_err_t autotune_save_to_nvs(autotune_handle_t handle, autotune_loop_t loop);

/**
 * @brief Load tuning results from NVS
 *
 * @param[in] loop      Which loop to load
 * @param[out] result   Returns saved results
 *
 * @return ESP_OK on success
 */
esp_err_t autotune_load_from_nvs(autotune_loop_t loop,
                                 autotune_result_t *result);

/**
 * @brief Get default configuration
 *
 * @return Default configuration
 */
autotune_config_t autotune_get_default_config(void);

#ifdef __cplusplus
}
#endif

#endif /* RELAY_FEEDBACK_H */
