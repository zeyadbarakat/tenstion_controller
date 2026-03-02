/**
 * @file hx711.h
 * @brief HX711 24-bit ADC Load Cell Amplifier Driver
 *
 * This driver provides an interface for the HX711 load cell amplifier:
 * - 24-bit resolution ADC readings
 * - Tare (zero) and span calibration with NVS storage
 * - Multi-stage filtering (moving average + median)
 * - Sensor disconnect detection
 * - Thread-safe operation
 *
 * HX711 Communication Protocol:
 * - No SPI/I2C - uses custom serial protocol
 * - DOUT goes low when data is ready
 * - 24 clock pulses to read data
 * - 1-3 additional pulses to set gain for next reading
 */

#ifndef HX711_H
#define HX711_H

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
 * @brief HX711 gain and channel selection
 */
typedef enum {
  HX711_GAIN_128_CH_A = 1, /**< Channel A, Gain 128 (25 pulses) */
  HX711_GAIN_32_CH_B = 2,  /**< Channel B, Gain 32 (26 pulses) */
  HX711_GAIN_64_CH_A = 3,  /**< Channel A, Gain 64 (27 pulses) */
} hx711_gain_t;

/**
 * @brief HX711 configuration structure
 */
typedef struct {
  int gpio_data;       /**< GPIO for DOUT (data) pin */
  int gpio_clock;      /**< GPIO for PD_SCK (clock) pin */
  hx711_gain_t gain;   /**< Gain and channel selection */
  uint8_t filter_size; /**< Moving average filter window (1-50) */
  uint8_t median_size; /**< Median filter window (1-9, odd only) */
} hx711_config_t;

/**
 * @brief Calibration data structure
 */
typedef struct {
  int32_t offset; /**< Zero offset (raw value at no load) */
  float scale;    /**< Scale factor (units per raw count) */
  bool valid;     /**< True if calibration is valid */
} hx711_calibration_t;

/**
 * @brief HX711 handle (opaque pointer)
 */
typedef struct hx711_s *hx711_handle_t;

/*******************************************************************************
 * Public API Functions
 ******************************************************************************/

/**
 * @brief Initialize HX711 driver
 *
 * @param[in] config    HX711 configuration
 * @param[out] handle   Returns HX711 handle on success
 *
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG: Invalid parameters
 *      - ESP_ERR_NO_MEM: Memory allocation failed
 */
esp_err_t hx711_init(const hx711_config_t *config, hx711_handle_t *handle);

/**
 * @brief Deinitialize HX711 and release resources
 *
 * @param[in] handle    HX711 handle
 *
 * @return ESP_OK on success
 */
esp_err_t hx711_deinit(hx711_handle_t handle);

/**
 * @brief Check if data is ready to read
 *
 * Data is ready when DOUT pin is low.
 *
 * @param[in] handle    HX711 handle
 *
 * @return true if data ready, false otherwise
 */
bool hx711_is_ready(hx711_handle_t handle);

/**
 * @brief Wait for data to be ready
 *
 * Blocks until DOUT goes low or timeout expires.
 *
 * @param[in] handle        HX711 handle
 * @param[in] timeout_ms    Maximum time to wait
 *
 * @return ESP_OK if ready, ESP_ERR_TIMEOUT if timeout
 */
esp_err_t hx711_wait_ready(hx711_handle_t handle, uint32_t timeout_ms);

/**
 * @brief Read raw 24-bit value from HX711
 *
 * Reads one sample directly from the ADC.
 *
 * @param[in] handle    HX711 handle
 * @param[out] value    Returns signed 24-bit value
 *
 * @return ESP_OK on success
 */
esp_err_t hx711_read_raw(hx711_handle_t handle, int32_t *value);

/**
 * @brief Read and filter multiple samples
 *
 * Reads multiple samples and applies configured filtering.
 *
 * @param[in] handle    HX711 handle
 * @param[out] value    Returns filtered value
 *
 * @return ESP_OK on success
 */
esp_err_t hx711_read_filtered(hx711_handle_t handle, int32_t *value);

/**
 * @brief Get weight in kilograms
 *
 * Reads filtered value and applies calibration.
 *
 * @param[in] handle    HX711 handle
 * @param[out] weight   Returns weight in kg
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not calibrated
 */
esp_err_t hx711_get_weight_kg(hx711_handle_t handle, float *weight);

/**
 * @brief Update weight reading (call periodically)
 *
 * Non-blocking update for use in control loops.
 * Returns the last calculated weight.
 *
 * @param[in] handle    HX711 handle
 *
 * @return Current weight in kg (or 0 if not ready/calibrated)
 */
float hx711_update(hx711_handle_t handle);

/**
 * @brief Perform tare (zero) calibration
 *
 * Reads N samples and stores average as zero offset.
 *
 * @param[in] handle    HX711 handle
 * @param[in] samples   Number of samples to average (10-100)
 *
 * @return ESP_OK on success
 */
esp_err_t hx711_tare(hx711_handle_t handle, uint8_t samples);

/**
 * @brief Set scale factor with known weight
 *
 * Call this after tare(), with a known weight on the load cell.
 *
 * @param[in] handle            HX711 handle
 * @param[in] known_weight_kg   Known weight in kg
 * @param[in] samples           Number of samples to average
 *
 * @return ESP_OK on success
 */
esp_err_t hx711_calibrate_scale(hx711_handle_t handle, float known_weight_kg,
                                uint8_t samples);

/**
 * @brief Set calibration values directly
 *
 * @param[in] handle    HX711 handle
 * @param[in] offset    Zero offset
 * @param[in] scale     Scale factor
 *
 * @return ESP_OK on success
 */
esp_err_t hx711_set_calibration(hx711_handle_t handle, int32_t offset,
                                float scale);

/**
 * @brief Get current calibration values
 *
 * @param[in] handle        HX711 handle
 * @param[out] calibration  Returns calibration data
 *
 * @return ESP_OK on success
 */
esp_err_t hx711_get_calibration(hx711_handle_t handle,
                                hx711_calibration_t *calibration);

/**
 * @brief Save calibration to NVS
 *
 * Stores offset and scale factor persistently.
 *
 * @param[in] handle    HX711 handle
 *
 * @return ESP_OK on success, ESP_ERR_NVS_* on NVS errors
 */
esp_err_t hx711_save_calibration(hx711_handle_t handle);

/**
 * @brief Load calibration from NVS
 *
 * Loads previously saved calibration values.
 *
 * @param[in] handle    HX711 handle
 *
 * @return ESP_OK on success, ESP_ERR_NVS_NOT_FOUND if no data
 */
esp_err_t hx711_load_calibration(hx711_handle_t handle);

/**
 * @brief Check if sensor is connected
 *
 * Returns false if readings are stuck or out of range.
 *
 * @param[in] handle    HX711 handle
 *
 * @return true if sensor appears connected
 */
bool hx711_is_connected(hx711_handle_t handle);

/**
 * @brief Power down HX711
 *
 * Puts HX711 into low-power mode by holding SCK high.
 *
 * @param[in] handle    HX711 handle
 *
 * @return ESP_OK on success
 */
esp_err_t hx711_power_down(hx711_handle_t handle);

/**
 * @brief Power up HX711
 *
 * Wakes HX711 from power-down mode.
 *
 * @param[in] handle    HX711 handle
 *
 * @return ESP_OK on success
 */
esp_err_t hx711_power_up(hx711_handle_t handle);

/**
 * @brief Create default HX711 configuration
 *
 * @return Default configuration structure
 */
hx711_config_t hx711_get_default_config(void);

/**
 * @brief Set EMA smoothing factor at runtime
 *
 * @param[in] handle    HX711 handle
 * @param[in] alpha     EMA factor (0.01 - 1.0). Lower = smoother.
 */
void hx711_set_ema_alpha(hx711_handle_t handle, float alpha);

/**
 * @brief Set moving average filter window at runtime
 *
 * @param[in] handle    HX711 handle
 * @param[in] size      Window size (1-50)
 */
void hx711_set_filter_size(hx711_handle_t handle, uint8_t size);

#ifdef __cplusplus
}
#endif

#endif /* HX711_H */
