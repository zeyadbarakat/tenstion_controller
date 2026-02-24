/**
 * @file motor_pwm.c
 * @brief DC Motor Controller Implementation using ESP32-S3 LEDC PWM
 *
 * LEDC (LED Control) Peripheral:
 * =============================
 * The ESP32-S3 LEDC peripheral provides hardware PWM generation with:
 * - Multiple independent channels
 * - Configurable frequency and resolution
 * - Hardware-based duty cycle updates
 * - Fade functionality (not used here for direct control)
 *
 * PWM Frequency vs Resolution Trade-off:
 * =====================================
 * For 80MHz APB clock:
 *
 * Resolution | Max Frequency | Steps
 * -----------+---------------+------
 * 10-bit     | ~78 kHz       | 1024
 * 12-bit     | ~19.5 kHz     | 4096
 * 14-bit     | ~4.8 kHz      | 16384
 *
 * We use 25kHz @ 10-bit resolution:
 * - Above human hearing range (silent operation)
 * - Sufficient resolution (1023 steps = ~0.1% per step)
 * - Compatible with most motor drivers
 *
 * Deadband Compensation:
 * =====================
 * Motor drivers and motors have static friction that requires
 * a minimum PWM before the motor starts moving. This is the "deadband".
 * We add this offset to low duty cycles to ensure motor response.
 */

#include "motor_pwm.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <math.h>
#include <string.h>


static const char *TAG = "motor";

/*******************************************************************************
 * Private Definitions
 ******************************************************************************/

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0

#define MAX_DUTY_10BIT 1023
#define DEFAULT_RAMP_RATE 100.0f // % per second
#define MAX_DEADBAND 10.0f       // Max deadband percentage

/*******************************************************************************
 * Private Structures
 ******************************************************************************/

struct motor_s {
  // Configuration
  gpio_num_t gpio_pwm;
  gpio_num_t gpio_dir;
  uint32_t max_duty;
  float ramp_rate;
  float deadband_percent;

  // Current state
  float current_speed;   // Current speed in percent
  float target_speed;    // Target speed for ramping
  uint32_t current_duty; // Current duty cycle
  motor_direction_t direction;
  motor_state_t state;
  bool enabled;

  // Ramping
  bool ramping;
  int64_t ramp_start_time;
  float ramp_start_speed;
  uint32_t ramp_duration_ms;

  // Thread safety
  SemaphoreHandle_t mutex;
};

/*******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/

static uint32_t speed_to_duty(motor_handle_t handle, float speed_percent);
static float duty_to_speed(motor_handle_t handle, uint32_t duty);
static void apply_motor_output(motor_handle_t handle, uint32_t duty,
                               bool forward);

/*******************************************************************************
 * Public API Implementation
 ******************************************************************************/

motor_config_t motor_get_default_config(void) {
  motor_config_t config = {
      .gpio_pwm = 15,
      .gpio_dir = 16,
      .pwm_freq_hz = 25000,     // 25 kHz
      .pwm_resolution = 10,     // 10-bit (0-1023)
      .ramp_rate = 100.0f,      // 100% per second
      .deadband_percent = 0.0f, // No deadband by default
  };
  return config;
}

esp_err_t motor_init(const motor_config_t *config, motor_handle_t *handle) {
  if (config == NULL || handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI(TAG, "Initializing motor: PWM=%d, DIR=%d, Freq=%lu Hz",
           config->gpio_pwm, config->gpio_dir,
           (unsigned long)config->pwm_freq_hz);

  // Allocate structure
  struct motor_s *motor = calloc(1, sizeof(struct motor_s));
  if (motor == NULL) {
    ESP_LOGE(TAG, "Failed to allocate motor structure");
    return ESP_ERR_NO_MEM;
  }

  // Store configuration
  motor->gpio_pwm = config->gpio_pwm;
  motor->gpio_dir = config->gpio_dir;
  motor->max_duty = (1 << config->pwm_resolution) - 1;
  motor->ramp_rate =
      config->ramp_rate > 0 ? config->ramp_rate : DEFAULT_RAMP_RATE;
  motor->deadband_percent = config->deadband_percent;
  motor->state = MOTOR_STATE_IDLE;

  // Create mutex
  motor->mutex = xSemaphoreCreateMutex();
  if (motor->mutex == NULL) {
    ESP_LOGE(TAG, "Failed to create mutex");
    free(motor);
    return ESP_ERR_NO_MEM;
  }

  // Configure LEDC timer
  ledc_timer_config_t timer_conf = {
      .speed_mode = LEDC_MODE,
      .timer_num = LEDC_TIMER,
      .duty_resolution = config->pwm_resolution,
      .freq_hz = config->pwm_freq_hz,
      .clk_cfg = LEDC_AUTO_CLK,
  };

  esp_err_t ret = ledc_timer_config(&timer_conf);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(ret));
    vSemaphoreDelete(motor->mutex);
    free(motor);
    return ret;
  }

  // Configure LEDC channel
  ledc_channel_config_t channel_conf = {
      .speed_mode = LEDC_MODE,
      .channel = LEDC_CHANNEL,
      .timer_sel = LEDC_TIMER,
      .intr_type = LEDC_INTR_DISABLE,
      .gpio_num = config->gpio_pwm,
      .duty = 0,
      .hpoint = 0,
  };

  ret = ledc_channel_config(&channel_conf);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure LEDC channel: %s", esp_err_to_name(ret));
    vSemaphoreDelete(motor->mutex);
    free(motor);
    return ret;
  }

  // Configure direction GPIO
  gpio_config_t io_conf = {
      .intr_type = GPIO_INTR_DISABLE,
      .mode = GPIO_MODE_OUTPUT,
      .pin_bit_mask = (1ULL << config->gpio_dir),
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .pull_up_en = GPIO_PULLUP_DISABLE,
  };
  gpio_config(&io_conf);
  gpio_set_level(config->gpio_dir, 0); // Default forward

  motor->enabled = true;
  motor->direction = MOTOR_DIR_STOPPED;

  ESP_LOGI(TAG, "Motor initialized: max_duty=%lu, resolution=%d-bit",
           (unsigned long)motor->max_duty, config->pwm_resolution);

  *handle = motor;
  return ESP_OK;
}

esp_err_t motor_deinit(motor_handle_t handle) {
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  // Stop motor first
  motor_emergency_stop(handle);

  // Stop LEDC
  ledc_stop(LEDC_MODE, LEDC_CHANNEL, 0);

  vSemaphoreDelete(handle->mutex);
  free(handle);

  ESP_LOGI(TAG, "Motor deinitialized");
  return ESP_OK;
}

esp_err_t motor_set_speed(motor_handle_t handle, float speed_percent) {
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  // Clamp speed to valid range
  if (speed_percent > 100.0f)
    speed_percent = 100.0f;
  if (speed_percent < -100.0f)
    speed_percent = -100.0f;

  xSemaphoreTake(handle->mutex, portMAX_DELAY);

  if (!handle->enabled) {
    xSemaphoreGive(handle->mutex);
    return ESP_ERR_INVALID_STATE;
  }

  // Determine direction
  bool forward = (speed_percent >= 0);
  float abs_speed = fabsf(speed_percent);

  // Convert to duty cycle
  uint32_t duty = speed_to_duty(handle, abs_speed);

  // Apply to hardware
  apply_motor_output(handle, duty, forward);

  // Update state
  handle->current_speed = speed_percent;
  handle->target_speed = speed_percent;
  handle->current_duty = duty;
  handle->ramping = false;

  if (abs_speed > 0.1f) {
    handle->direction = forward ? MOTOR_DIR_FORWARD : MOTOR_DIR_REVERSE;
    handle->state = MOTOR_STATE_RUNNING;
  } else {
    handle->direction = MOTOR_DIR_STOPPED;
    handle->state = MOTOR_STATE_IDLE;
  }

  xSemaphoreGive(handle->mutex);

  ESP_LOGD(TAG, "Speed set: %.1f%% (duty=%lu)", speed_percent,
           (unsigned long)duty);

  return ESP_OK;
}

esp_err_t motor_set_duty(motor_handle_t handle, uint32_t duty, bool forward) {
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (duty > handle->max_duty) {
    duty = handle->max_duty;
  }

  xSemaphoreTake(handle->mutex, portMAX_DELAY);

  if (!handle->enabled) {
    xSemaphoreGive(handle->mutex);
    return ESP_ERR_INVALID_STATE;
  }

  apply_motor_output(handle, duty, forward);

  handle->current_duty = duty;
  handle->current_speed =
      forward ? duty_to_speed(handle, duty) : -duty_to_speed(handle, duty);
  handle->target_speed = handle->current_speed;
  handle->direction = (duty > 0)
                          ? (forward ? MOTOR_DIR_FORWARD : MOTOR_DIR_REVERSE)
                          : MOTOR_DIR_STOPPED;
  handle->state = (duty > 0) ? MOTOR_STATE_RUNNING : MOTOR_STATE_IDLE;
  handle->ramping = false;

  xSemaphoreGive(handle->mutex);

  return ESP_OK;
}

float motor_get_speed(motor_handle_t handle) {
  if (handle == NULL) {
    return 0.0f;
  }

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  float speed = handle->current_speed;
  xSemaphoreGive(handle->mutex);

  return speed;
}

uint32_t motor_get_duty(motor_handle_t handle) {
  if (handle == NULL) {
    return 0;
  }

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  uint32_t duty = handle->current_duty;
  xSemaphoreGive(handle->mutex);

  return duty;
}

motor_direction_t motor_get_direction(motor_handle_t handle) {
  if (handle == NULL) {
    return MOTOR_DIR_STOPPED;
  }

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  motor_direction_t dir = handle->direction;
  xSemaphoreGive(handle->mutex);

  return dir;
}

motor_state_t motor_get_state(motor_handle_t handle) {
  if (handle == NULL) {
    return MOTOR_STATE_IDLE;
  }

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  motor_state_t state = handle->state;
  xSemaphoreGive(handle->mutex);

  return state;
}

esp_err_t motor_brake(motor_handle_t handle) {
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI(TAG, "Motor brake activated");

  xSemaphoreTake(handle->mutex, portMAX_DELAY);

  // Set PWM to 0
  ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
  ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

  handle->current_duty = 0;
  handle->current_speed = 0;
  handle->target_speed = 0;
  handle->direction = MOTOR_DIR_STOPPED;
  handle->state = MOTOR_STATE_BRAKING;
  handle->ramping = false;

  xSemaphoreGive(handle->mutex);

  return ESP_OK;
}

esp_err_t motor_coast(motor_handle_t handle) {
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI(TAG, "Motor coast (freewheel)");

  xSemaphoreTake(handle->mutex, portMAX_DELAY);

  // Set PWM to 0
  ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
  ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

  handle->current_duty = 0;
  handle->current_speed = 0;
  handle->target_speed = 0;
  handle->direction = MOTOR_DIR_STOPPED;
  handle->state = MOTOR_STATE_IDLE;
  handle->ramping = false;

  xSemaphoreGive(handle->mutex);

  return ESP_OK;
}

esp_err_t motor_soft_start(motor_handle_t handle, float target_percent,
                           uint32_t ramp_time_ms) {
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (target_percent > 100.0f)
    target_percent = 100.0f;
  if (target_percent < -100.0f)
    target_percent = -100.0f;

  ESP_LOGI(TAG, "Soft start to %.1f%% over %lu ms", target_percent,
           (unsigned long)ramp_time_ms);

  xSemaphoreTake(handle->mutex, portMAX_DELAY);

  if (!handle->enabled) {
    xSemaphoreGive(handle->mutex);
    return ESP_ERR_INVALID_STATE;
  }

  handle->target_speed = target_percent;
  handle->ramp_start_speed = handle->current_speed;
  handle->ramp_start_time = esp_timer_get_time();
  handle->ramp_duration_ms = ramp_time_ms;
  handle->ramping = true;
  handle->state = MOTOR_STATE_RAMPING;

  xSemaphoreGive(handle->mutex);

  return ESP_OK;
}

esp_err_t motor_emergency_stop(motor_handle_t handle) {
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGW(TAG, "EMERGENCY STOP");

  // Bypass mutex for emergency - direct hardware access
  ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
  ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

  xSemaphoreTake(handle->mutex, portMAX_DELAY);

  handle->current_duty = 0;
  handle->current_speed = 0;
  handle->target_speed = 0;
  handle->direction = MOTOR_DIR_STOPPED;
  handle->state = MOTOR_STATE_IDLE;
  handle->ramping = false;

  xSemaphoreGive(handle->mutex);

  return ESP_OK;
}

esp_err_t motor_update(motor_handle_t handle) {
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  xSemaphoreTake(handle->mutex, portMAX_DELAY);

  if (!handle->ramping || !handle->enabled) {
    xSemaphoreGive(handle->mutex);
    return ESP_OK;
  }

  // Calculate ramp progress
  int64_t now = esp_timer_get_time();
  int64_t elapsed_us = now - handle->ramp_start_time;
  float elapsed_ms = (float)elapsed_us / 1000.0f;

  if (elapsed_ms >= handle->ramp_duration_ms) {
    // Ramp complete
    handle->current_speed = handle->target_speed;
    handle->ramping = false;
    handle->state = (fabsf(handle->target_speed) > 0.1f) ? MOTOR_STATE_RUNNING
                                                         : MOTOR_STATE_IDLE;
  } else {
    // Linear interpolation
    float progress = elapsed_ms / (float)handle->ramp_duration_ms;
    float speed_delta = handle->target_speed - handle->ramp_start_speed;
    handle->current_speed = handle->ramp_start_speed + (speed_delta * progress);
  }

  // Apply the calculated speed
  bool forward = (handle->current_speed >= 0);
  float abs_speed = fabsf(handle->current_speed);
  uint32_t duty = speed_to_duty(handle, abs_speed);

  apply_motor_output(handle, duty, forward);
  handle->current_duty = duty;

  if (abs_speed > 0.1f) {
    handle->direction = forward ? MOTOR_DIR_FORWARD : MOTOR_DIR_REVERSE;
  } else {
    handle->direction = MOTOR_DIR_STOPPED;
  }

  xSemaphoreGive(handle->mutex);

  return ESP_OK;
}

esp_err_t motor_enable(motor_handle_t handle, bool enable) {
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  xSemaphoreTake(handle->mutex, portMAX_DELAY);

  if (!enable && handle->enabled) {
    // Disabling - stop motor first
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    handle->current_duty = 0;
    handle->current_speed = 0;
    handle->direction = MOTOR_DIR_STOPPED;
    handle->state = MOTOR_STATE_IDLE;
  }

  handle->enabled = enable;

  xSemaphoreGive(handle->mutex);

  ESP_LOGI(TAG, "Motor %s", enable ? "enabled" : "disabled");

  return ESP_OK;
}

bool motor_is_enabled(motor_handle_t handle) {
  if (handle == NULL) {
    return false;
  }

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  bool enabled = handle->enabled;
  xSemaphoreGive(handle->mutex);

  return enabled;
}

esp_err_t motor_set_ramp_rate(motor_handle_t handle, float rate) {
  if (handle == NULL || rate <= 0) {
    return ESP_ERR_INVALID_ARG;
  }

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  handle->ramp_rate = rate;
  xSemaphoreGive(handle->mutex);

  return ESP_OK;
}

esp_err_t motor_set_deadband(motor_handle_t handle, float deadband_percent) {
  if (handle == NULL || deadband_percent < 0 ||
      deadband_percent > MAX_DEADBAND) {
    return ESP_ERR_INVALID_ARG;
  }

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  handle->deadband_percent = deadband_percent;
  xSemaphoreGive(handle->mutex);

  return ESP_OK;
}

/*******************************************************************************
 * Private Function Implementations
 ******************************************************************************/

/**
 * @brief Convert speed percentage to duty cycle with deadband
 *
 * Applies deadband compensation: for low speeds, adds minimum duty
 * to overcome static friction.
 */
static uint32_t speed_to_duty(motor_handle_t handle, float speed_percent) {
  if (speed_percent < 0.1f) {
    return 0; // Below threshold, stop
  }

  // Apply deadband compensation
  float effective_speed = speed_percent;
  if (handle->deadband_percent > 0 && speed_percent > 0.1f) {
    // Map 0-100% to deadband-100%
    float range = 100.0f - handle->deadband_percent;
    effective_speed =
        handle->deadband_percent + (speed_percent * range / 100.0f);
  }

  // Convert to duty
  uint32_t duty = (uint32_t)((effective_speed / 100.0f) * handle->max_duty);

  if (duty > handle->max_duty) {
    duty = handle->max_duty;
  }

  return duty;
}

/**
 * @brief Convert duty cycle to speed percentage
 */
static float duty_to_speed(motor_handle_t handle, uint32_t duty) {
  return (float)duty * 100.0f / (float)handle->max_duty;
}

/**
 * @brief Apply motor output to hardware
 */
static void apply_motor_output(motor_handle_t handle, uint32_t duty,
                               bool forward) {
  // Set direction
  gpio_set_level(handle->gpio_dir, forward ? 0 : 1);

  // Set PWM duty
  ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
  ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}
