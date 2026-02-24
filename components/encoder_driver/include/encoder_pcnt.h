/**
 * @file encoder_pcnt.h
 * @brief Quadrature Encoder Driver using ESP32-S3 PCNT Peripheral
 *
 * This driver provides hardware-based quadrature decoding using the ESP32-S3's
 * Pulse Counter (PCNT) peripheral. It supports:
 * - 4x quadrature counting (counting all edges on both channels)
 * - Bidirectional counting with overflow/underflow handling
 * - Filtered RPM calculation with moving average
 * - Thread-safe operation
 *
 * @note For a 600 PPR encoder, the effective resolution is 2400 counts/rev
 *       in 4x quadrature mode.
 */

#ifndef ENCODER_PCNT_H
#define ENCODER_PCNT_H

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
 * @brief Encoder rotation direction
 */
typedef enum {
  ENCODER_DIR_CW = 1,     /**< Clockwise rotation (positive) */
  ENCODER_DIR_CCW = -1,   /**< Counter-clockwise rotation (negative) */
  ENCODER_DIR_STOPPED = 0 /**< No rotation detected */
} encoder_direction_t;

/**
 * @brief Encoder configuration structure
 */
typedef struct {
  int gpio_a;          /**< GPIO for encoder channel A */
  int gpio_b;          /**< GPIO for encoder channel B */
  uint32_t ppr;        /**< Pulses per revolution (before 4x multiplication) */
  uint8_t filter_size; /**< Moving average filter window size (1-20) */
  int16_t count_high;  /**< High limit for PCNT counter (triggers overflow) */
  int16_t count_low;   /**< Low limit for PCNT counter (triggers underflow) */
} encoder_config_t;

/**
 * @brief Encoder handle (opaque pointer)
 */
typedef struct encoder_s *encoder_handle_t;

/*******************************************************************************
 * Public API Functions
 ******************************************************************************/

/**
 * @brief Initialize encoder with PCNT quadrature decoding
 *
 * Configures the PCNT peripheral for 4x quadrature decoding mode:
 * - Channel A: count on rising edge (increment), falling edge (decrement)
 * - Channel B: provides direction information
 *
 * @param[in] config    Encoder configuration
 * @param[out] handle   Returns encoder handle on success
 *
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG: Invalid parameters
 *      - ESP_ERR_NO_MEM: Memory allocation failed
 *      - ESP_FAIL: PCNT configuration failed
 */
esp_err_t encoder_init(const encoder_config_t *config,
                       encoder_handle_t *handle);

/**
 * @brief Deinitialize encoder and release resources
 *
 * @param[in] handle    Encoder handle
 *
 * @return ESP_OK on success
 */
esp_err_t encoder_deinit(encoder_handle_t handle);

/**
 * @brief Get current accumulated pulse count
 *
 * Returns the total accumulated count including overflow handling.
 * The count is signed: positive for CW, negative for CCW.
 *
 * @param[in] handle    Encoder handle
 *
 * @return Current pulse count (thread-safe)
 */
int32_t encoder_get_count(encoder_handle_t handle);

/**
 * @brief Reset encoder count to zero
 *
 * Clears both the PCNT counter and the accumulated overflow count.
 *
 * @param[in] handle    Encoder handle
 *
 * @return ESP_OK on success
 */
esp_err_t encoder_reset_count(encoder_handle_t handle);

/**
 * @brief Get filtered RPM value
 *
 * Returns the current motor speed in RPM, filtered using a moving average.
 * Positive values indicate CW rotation, negative for CCW.
 *
 * Formula: RPM = (delta_count / CPR) * (60 / delta_time_s)
 * Where CPR = PPR * 4 (quadrature multiplication)
 *
 * @param[in] handle    Encoder handle
 *
 * @return Current RPM (filtered, signed)
 */
float encoder_get_rpm(encoder_handle_t handle);

/**
 * @brief Get raw (unfiltered) RPM value
 *
 * @param[in] handle    Encoder handle
 *
 * @return Current RPM (unfiltered, signed)
 */
float encoder_get_rpm_raw(encoder_handle_t handle);

/**
 * @brief Get rotation direction
 *
 * @param[in] handle    Encoder handle
 *
 * @return ENCODER_DIR_CW, ENCODER_DIR_CCW, or ENCODER_DIR_STOPPED
 */
encoder_direction_t encoder_get_direction(encoder_handle_t handle);

/**
 * @brief Update RPM calculation
 *
 * This function should be called periodically (e.g., every 10-20ms) from a
 * dedicated task to calculate the current RPM based on count changes.
 *
 * @param[in] handle    Encoder handle
 *
 * @return ESP_OK on success
 */
esp_err_t encoder_update(encoder_handle_t handle);

/**
 * @brief Check if encoder is receiving pulses
 *
 * Returns false if no pulses have been received for a specified timeout.
 * Useful for detecting encoder disconnection or motor stall.
 *
 * @param[in] handle        Encoder handle
 * @param[in] timeout_ms    Timeout in milliseconds
 *
 * @return true if pulses received within timeout, false otherwise
 */
bool encoder_is_active(encoder_handle_t handle, uint32_t timeout_ms);

/**
 * @brief Get time since last pulse
 *
 * @param[in] handle    Encoder handle
 *
 * @return Time in milliseconds since last pulse detected
 */
uint32_t encoder_get_idle_time_ms(encoder_handle_t handle);

/**
 * @brief Create default encoder configuration
 *
 * @return Default configuration structure
 */
encoder_config_t encoder_get_default_config(void);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_PCNT_H */
