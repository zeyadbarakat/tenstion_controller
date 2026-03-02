/**
 * @file control_manager.c
 * @brief Central Control Manager Implementation
 *
 * Control Loop Architecture:
 * =========================
 *
 *     ┌──────────────────────────────────────────────────────────┐
 *     │                    CONTROL MANAGER                       │
 *     ├──────────────────────────────────────────────────────────┤
 *     │                                                          │
 *     │  Tension     ┌──────────┐     Speed      ┌──────────┐    │
 *     │  Setpoint───►│ Tension  ├──────Setpoint─►│  Speed   ├───►│PWM
 *     │              │    PI    │                │    PI    │    │
 *     │          ┌──►│Controller│◄───────────────│Controller│◄─┐ │
 *     │          │   └──────────┘                └──────────┘  │ │
 *     │          │                                             │ │
 *     │       Tension                                       Speed│
 *     │       Feedback                                   Feedback│
 *     │          │                                             │ │
 *     │   ┌──────┴───────┐                           ┌─────────┴┐│
 *     │   │   HX711      │                           │  PCNT    ││
 *     │   │  Load Cell   │                           │ Encoder  ││
 *     │   └──────────────┘                           └──────────┘│
 *     │                                                          │
 *     └──────────────────────────────────────────────────────────┘
 *
 * FreeRTOS Tasks:
 * ==============
 * 1. Control Task (10Hz) - Runs control loops
 * 2. UI Task (5Hz) - Updates display, handles input
 * 3. Button Task (50Hz) - Debounces buttons
 */

#include "control_manager.h"
#include "buttons.h"
#include "encoder_pcnt.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "hx711.h"
#include "lcd_menu.h"
#include "logger.h"
#include "motor_pwm.h"
#include "nvs_flash.h"
#include "pi_controller.h"
#include "relay_feedback.h"
#include "safety.h"
#include "sdkconfig.h"
#include <math.h>
#include <string.h>

static const char *TAG = "ctrl_mgr";

/*******************************************************************************
 * Private Structures
 ******************************************************************************/

struct control_manager_s {
  // Hardware handles
  encoder_handle_t encoder;
  hx711_handle_t loadcell;
  motor_handle_t motor;

  // Control handles
  pi_controller_t speed_pi;
  pi_controller_t tension_pi;
  autotune_handle_t autotuner;
  safety_handle_t safety;
  logger_handle_t logger;

  // UI handles
  buttons_handle_t buttons;
  lcd_handle_t lcd;

  // Configuration
  control_manager_config_t config;

  // State
  control_mode_t mode;
  float tension_setpoint;
  float speed_setpoint;
  float tension_step; /* Configurable step for ▲/▼ keys */
  system_status_t status;
  int64_t start_time_us;

  // Tasks
  TaskHandle_t control_task;
  TaskHandle_t ui_task;
  bool running;

  // Thread safety
  SemaphoreHandle_t mutex;
};

/*******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/

static void control_task(void *arg);
static void ui_task(void *arg);
static void on_button_event(button_event_t event, void *user_data);
static void on_lcd_command(uint8_t command, float value, void *user_data);
static void on_menu_command(lcd_command_id_t cmd, float value, void *user_data);
static void run_control_loop(control_manager_handle_t mgr);

/*******************************************************************************
 * Public API Implementation
 ******************************************************************************/

control_manager_config_t control_manager_get_default_config(void) {
  control_manager_config_t config = {
      .encoder_a_gpio = 4,
      .encoder_b_gpio = 5,
      .hx711_data_gpio = CONFIG_HX711_GPIO_DATA,
      .hx711_clock_gpio = CONFIG_HX711_GPIO_CLOCK,
      .motor_pwm_gpio = 15,
      .motor_dir_gpio = 16,
      .button_run_gpio = 10,
      .button_stop_gpio = 11,
      .button_estop_gpio = 12,
      .control_period_ms = 100, // 10Hz
      .ui_period_ms = 200,      // 5Hz
      .default_tension_setpoint = 5.0f,
  };
  return config;
}

esp_err_t control_manager_init(control_manager_handle_t *handle,
                               const control_manager_config_t *config) {
  if (handle == NULL || config == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI(TAG, "Initializing control manager...");

  struct control_manager_s *mgr = calloc(1, sizeof(struct control_manager_s));
  if (mgr == NULL) {
    return ESP_ERR_NO_MEM;
  }

  memcpy(&mgr->config, config, sizeof(control_manager_config_t));
  mgr->mutex = xSemaphoreCreateMutex();
  if (mgr->mutex == NULL) {
    free(mgr);
    return ESP_ERR_NO_MEM;
  }

  esp_err_t ret;

  // === Initialize Encoder ===
  encoder_config_t enc_cfg = encoder_get_default_config();
  enc_cfg.gpio_a = config->encoder_a_gpio;
  enc_cfg.gpio_b = config->encoder_b_gpio;

  // Read PPR from NVS if previously saved via web config
  {
    nvs_handle_t nvs_enc;
    if (nvs_open("config", NVS_READONLY, &nvs_enc) == ESP_OK) {
      uint16_t saved_ppr = 0;
      if (nvs_get_u16(nvs_enc, "ppr", &saved_ppr) == ESP_OK && saved_ppr > 0) {
        enc_cfg.ppr = saved_ppr;
        ESP_LOGI(TAG, "Loaded PPR from NVS: %u", saved_ppr);
      }
      nvs_close(nvs_enc);
    }
  }

  ret = encoder_init(&enc_cfg, &mgr->encoder);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Encoder init failed: %s", esp_err_to_name(ret));
  }

  // === Initialize Load Cell ===
  hx711_config_t hx_cfg = hx711_get_default_config();
  hx_cfg.gpio_data = config->hx711_data_gpio;
  hx_cfg.gpio_clock = config->hx711_clock_gpio;
  hx_cfg.filter_size = CONFIG_HX711_FILTER_SIZE;
  ret = hx711_init(&hx_cfg, &mgr->loadcell);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "HX711 init failed: %s", esp_err_to_name(ret));
  }

  // Load calibration from NVS
  hx711_load_calibration(mgr->loadcell);

  // Load filter settings from NVS (web-configured values)
  {
    nvs_handle_t nvs_filt;
    if (nvs_open("config", NVS_READONLY, &nvs_filt) == ESP_OK) {
      int32_t ema_val = 0, ma_val = 0;
      if (nvs_get_i32(nvs_filt, "ema_alpha", &ema_val) == ESP_OK &&
          ema_val > 0) {
        hx711_set_ema_alpha(mgr->loadcell, (float)ema_val / 100.0f);
      }
      if (nvs_get_i32(nvs_filt, "ma_window", &ma_val) == ESP_OK && ma_val > 0) {
        hx711_set_filter_size(mgr->loadcell, (uint8_t)ma_val);
      }
      nvs_close(nvs_filt);
    }
  }

  // === Initialize Motor ===
  motor_config_t mot_cfg = motor_get_default_config();
  mot_cfg.gpio_pwm = config->motor_pwm_gpio;
  mot_cfg.gpio_dir = config->motor_dir_gpio;
  ret = motor_init(&mot_cfg, &mgr->motor);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Motor init failed: %s", esp_err_to_name(ret));
  }

  // === Initialize PI Controllers ===
  float dt = config->control_period_ms / 1000.0f;

  // Speed PI: faster response
  pi_init(&mgr->speed_pi, 0.5f, 0.1f, dt, 0.0f, 100.0f);

  // Tension PI: slower, cascaded outer loop
  pi_init(&mgr->tension_pi, 0.3f, 0.05f, dt, -1000.0f, 1000.0f);

  // === Initialize Auto-tuner ===
  autotune_init(&mgr->autotuner);

  // === Initialize Safety Monitor ===
  safety_init(&mgr->safety);
  safety_load_limits(mgr->safety);

  // === Initialize Data Logger ===
  logger_init(&mgr->logger, 500); // 500 samples buffer

  // === Initialize Buttons ===
  button_config_t btn_cfg = buttons_get_default_config();
  btn_cfg.gpio_run = config->button_run_gpio;
  btn_cfg.gpio_stop = config->button_stop_gpio;
  btn_cfg.gpio_estop = config->button_estop_gpio;
  buttons_init(&mgr->buttons, &btn_cfg);
  buttons_register_callback(mgr->buttons, on_button_event, mgr);

  // === Initialize LCD (I2C 20x4 + 1x4 Keypad) ===
  lcd_config_t lcd_cfg = {
      .sda_gpio = CONFIG_LCD_GPIO_SDA,
      .scl_gpio = CONFIG_LCD_GPIO_SCL,
      .i2c_addr = CONFIG_LCD_I2C_ADDR,
      .key_gpios = {CONFIG_KEY_GPIO_1, CONFIG_KEY_GPIO_2, CONFIG_KEY_GPIO_3,
                    CONFIG_KEY_GPIO_4},
  };
  ret = lcd_init(&mgr->lcd, &lcd_cfg);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "LCD init failed: %s", esp_err_to_name(ret));
  }
  lcd_set_callback(mgr->lcd, on_lcd_command, mgr);
  lcd_set_menu_callback(mgr->lcd, on_menu_command, mgr);

  // Set defaults
  mgr->tension_setpoint = config->default_tension_setpoint;
  mgr->tension_step = 0.5f;
  mgr->mode = MODE_IDLE;
  mgr->start_time_us = esp_timer_get_time();

  // Load saved setpoint and PI gains from NVS if available
  nvs_handle_t nvs;
  if (nvs_open("config", NVS_READONLY, &nvs) == ESP_OK) {
    int32_t saved_sp = 0;
    if (nvs_get_i32(nvs, "setpoint", &saved_sp) == ESP_OK) {
      mgr->tension_setpoint = (float)saved_sp / 100.0f;
      ESP_LOGI(TAG, "Restored setpoint from NVS: %.2f kg",
               mgr->tension_setpoint);
    }

    // Load saved PI gains from previous auto-tune (x10000 precision)
    int32_t speed_kp_x = 0, speed_ki_x = 0;
    int32_t tension_kp_x = 0, tension_ki_x = 0;

    if (nvs_get_i32(nvs, "speed_kp", &speed_kp_x) == ESP_OK &&
        nvs_get_i32(nvs, "speed_ki", &speed_ki_x) == ESP_OK) {
      float skp = (float)speed_kp_x / 10000.0f;
      float ski = (float)speed_ki_x / 10000.0f;
      if (skp > 0.0001f && skp < 100.0f && ski > 0.00001f && ski < 50.0f) {
        pi_set_gains(&mgr->speed_pi, skp, ski);
        ESP_LOGI(TAG, "Restored speed PI from NVS: Kp=%.4f Ki=%.4f", skp, ski);
      }
    }

    if (nvs_get_i32(nvs, "tension_kp", &tension_kp_x) == ESP_OK &&
        nvs_get_i32(nvs, "tension_ki", &tension_ki_x) == ESP_OK) {
      float tkp = (float)tension_kp_x / 10000.0f;
      float tki = (float)tension_ki_x / 10000.0f;
      if (tkp > 0.0001f && tkp < 100.0f && tki > 0.00001f && tki < 50.0f) {
        pi_set_gains(&mgr->tension_pi, tkp, tki);
        ESP_LOGI(TAG, "Restored tension PI from NVS: Kp=%.4f Ki=%.4f", tkp,
                 tki);
      }
    }

    nvs_close(nvs);
  }

  ESP_LOGI(TAG, "Control manager initialized successfully");

  *handle = mgr;
  return ESP_OK;
}

esp_err_t control_manager_start(control_manager_handle_t handle) {
  if (handle == NULL)
    return ESP_ERR_INVALID_ARG;

  ESP_LOGI(TAG, "Starting control system...");

  handle->running = true;

  // Create control task (high priority)
  xTaskCreatePinnedToCore(control_task, "control", 4096, handle,
                          configMAX_PRIORITIES - 1, &handle->control_task,
                          1 // Core 1
  );

  // Create UI task (lower priority)
  xTaskCreatePinnedToCore(ui_task, "ui", 4096, handle, 5, &handle->ui_task,
                          0 // Core 0
  );

  ESP_LOGI(TAG, "Control system started");
  return ESP_OK;
}

void control_manager_stop(control_manager_handle_t handle) {
  if (handle == NULL)
    return;

  handle->running = false;

  // Stop motor immediately
  motor_emergency_stop(handle->motor);

  // Wait for tasks to exit
  vTaskDelay(pdMS_TO_TICKS(200));

  if (handle->control_task) {
    vTaskDelete(handle->control_task);
    handle->control_task = NULL;
  }
  if (handle->ui_task) {
    vTaskDelete(handle->ui_task);
    handle->ui_task = NULL;
  }

  ESP_LOGI(TAG, "Control system stopped");
}

void control_manager_deinit(control_manager_handle_t handle) {
  if (handle == NULL)
    return;

  control_manager_stop(handle);

  encoder_deinit(handle->encoder);
  hx711_deinit(handle->loadcell);
  motor_deinit(handle->motor);
  autotune_deinit(handle->autotuner);
  safety_deinit(handle->safety);
  logger_deinit(handle->logger);
  buttons_deinit(handle->buttons);
  lcd_deinit(handle->lcd);

  vSemaphoreDelete(handle->mutex);
  free(handle);

  ESP_LOGI(TAG, "Control manager deinitialized");
}

void control_manager_get_status(control_manager_handle_t handle,
                                system_status_t *status) {
  if (handle == NULL || status == NULL)
    return;

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  memcpy(status, &handle->status, sizeof(system_status_t));
  xSemaphoreGive(handle->mutex);
}

esp_err_t control_manager_set_tension(control_manager_handle_t handle,
                                      float tension) {
  if (handle == NULL)
    return ESP_ERR_INVALID_ARG;

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  handle->tension_setpoint = tension;
  xSemaphoreGive(handle->mutex);

  // Save setpoint to NVS for persistence
  nvs_handle_t nvs;
  if (nvs_open("config", NVS_READWRITE, &nvs) == ESP_OK) {
    nvs_set_i32(nvs, "setpoint", (int32_t)(tension * 100.0f));
    nvs_commit(nvs);
    nvs_close(nvs);
  }

  ESP_LOGI(TAG, "Tension setpoint: %.2f kg (saved)", tension);
  return ESP_OK;
}

esp_err_t control_manager_set_mode(control_manager_handle_t handle,
                                   control_mode_t mode) {
  if (handle == NULL)
    return ESP_ERR_INVALID_ARG;

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  handle->mode = mode;

  // Reset controllers on mode change
  pi_reset(&handle->speed_pi);
  pi_reset(&handle->tension_pi);

  xSemaphoreGive(handle->mutex);

  ESP_LOGI(TAG, "Mode changed to %d", mode);
  return ESP_OK;
}

esp_err_t control_manager_run(control_manager_handle_t handle) {
  if (handle == NULL)
    return ESP_ERR_INVALID_ARG;

  if (!safety_is_safe_to_start(handle->safety)) {
    ESP_LOGW(TAG, "Cannot start - not safe");
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t ret = safety_request_start(handle->safety);
  if (ret == ESP_OK) {
    xSemaphoreTake(handle->mutex, portMAX_DELAY);
    handle->mode = MODE_AUTO_TENSION;
    pi_set_enabled(&handle->speed_pi, true);
    pi_set_enabled(&handle->tension_pi, true);
    xSemaphoreGive(handle->mutex);

    ESP_LOGI(TAG, "System RUN");
  }

  return ret;
}

void control_manager_stop_controlled(control_manager_handle_t handle) {
  if (handle == NULL)
    return;

  safety_request_stop(handle->safety);

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  pi_set_enabled(&handle->speed_pi, false);
  pi_set_enabled(&handle->tension_pi, false);
  xSemaphoreGive(handle->mutex);

  motor_soft_start(handle->motor, 0, 2000); // Ramp down over 2s

  ESP_LOGI(TAG, "Controlled stop initiated");
}

void control_manager_emergency_stop(control_manager_handle_t handle) {
  if (handle == NULL)
    return;

  safety_emergency_stop(handle->safety);
  motor_emergency_stop(handle->motor);

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  handle->mode = MODE_IDLE;
  pi_set_enabled(&handle->speed_pi, false);
  pi_set_enabled(&handle->tension_pi, false);
  xSemaphoreGive(handle->mutex);

  ESP_LOGW(TAG, "EMERGENCY STOP");
}

esp_err_t control_manager_reset_faults(control_manager_handle_t handle) {
  if (handle == NULL)
    return ESP_ERR_INVALID_ARG;

  // Try to reset button latch first (if hardware released)
  buttons_reset_estop(handle->buttons);

  esp_err_t ret = safety_reset_fault(handle->safety);
  if (ret == ESP_OK) {
    // Reset mode so jog/restart works after fault recovery
    handle->mode = MODE_IDLE;
    ESP_LOGI(TAG, "Faults cleared, mode reset to IDLE");
  }
  return ret;
}

void control_manager_start_tare(control_manager_handle_t handle) {
  if (handle == NULL)
    return;

  ESP_LOGI(TAG, "Starting tare calibration...");
  hx711_tare(handle->loadcell, 50);
  lcd_show_message(handle->lcd, "Tare complete", 2000);
}

void control_manager_start_calibration(control_manager_handle_t handle,
                                       float known_weight) {
  if (handle == NULL)
    return;

  ESP_LOGI(TAG, "Starting span calibration with %.2f kg", known_weight);
  hx711_calibrate_scale(handle->loadcell, known_weight, 50);
  hx711_save_calibration(handle->loadcell);
  lcd_show_message(handle->lcd, "Calibration saved", 2000);
}

void control_manager_start_autotune(control_manager_handle_t handle,
                                    bool speed_loop) {
  if (handle == NULL)
    return;

  if (!safety_is_safe_to_start(handle->safety)) {
    ESP_LOGW(TAG, "Cannot start auto-tune - not safe to run");
    lcd_show_message(handle->lcd, "Safety Not Ready!", 3000);
    return;
  }

  // Tuning order enforcement: speed loop must be tuned before tension
  if (!speed_loop) {
    // Check if load cell is calibrated (required for tension autotune)
    hx711_calibration_t cal;
    hx711_get_calibration(handle->loadcell, &cal);
    if (!cal.valid) {
      ESP_LOGW(TAG, "Load cell not calibrated! Tension autotune aborted.");
      lcd_show_message(handle->lcd, "Calibrate first!", 3000);
      return;
    }

    // Check if speed loop has valid tuning
    nvs_handle_t nvs;
    bool speed_tuned = false;
    if (nvs_open("config", NVS_READONLY, &nvs) == ESP_OK) {
      int32_t skp = 0;
      if (nvs_get_i32(nvs, "speed_kp", &skp) == ESP_OK && skp > 0) {
        speed_tuned = true;
      }
      nvs_close(nvs);
    }
    if (!speed_tuned) {
      ESP_LOGW(TAG, "Speed loop must be tuned BEFORE tension loop!");
      ESP_LOGW(TAG, "Run speed auto-tune first, then tension auto-tune.");
      lcd_show_message(handle->lcd, "Tune SPEED first!", 3000);
      return;
    }
  }

  esp_err_t ret = safety_request_start(handle->safety);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to transition to STARTING state for auto-tune");
    return;
  }

  autotune_config_t cfg = autotune_get_default_config();
  cfg.loop = speed_loop ? AUTOTUNE_SPEED_LOOP : AUTOTUNE_TENSION_LOOP;
  // Use 30% of max RPM for speed tune (safe operating point)
  cfg.setpoint = speed_loop ? 600.0f : handle->tension_setpoint;
  cfg.relay_amplitude = 20.0f;
  cfg.bias = 50.0f;

  autotune_start(handle->autotuner, &cfg);

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  handle->mode = speed_loop ? MODE_TUNING_SPEED : MODE_TUNING_TENSION;
  // Enable speed PI for tension tuning (inner loop must stay active)
  if (!speed_loop) {
    pi_set_enabled(&handle->speed_pi, true);
  }
  xSemaphoreGive(handle->mutex);

  ESP_LOGI(TAG, "Auto-tune started for %s loop (setpoint=%.0f)",
           speed_loop ? "speed" : "tension", cfg.setpoint);
}

void control_manager_jog(control_manager_handle_t handle, float speed_percent) {
  if (handle == NULL)
    return;

  xSemaphoreTake(handle->mutex, portMAX_DELAY);

  // Only allow jog when in idle mode
  if (handle->mode != MODE_IDLE && speed_percent != 0.0f) {
    ESP_LOGW(TAG, "Jog rejected - system not idle (mode=%d)", handle->mode);
    xSemaphoreGive(handle->mutex);
    return;
  }

  // Clamp speed to safe range
  if (speed_percent > 50.0f)
    speed_percent = 50.0f;
  if (speed_percent < -50.0f)
    speed_percent = -50.0f;

  xSemaphoreGive(handle->mutex);

  // Apply motor PWM directly
  if (handle->motor != NULL) {
    if (speed_percent == 0.0f) {
      motor_set_speed(handle->motor, 0.0f);
      ESP_LOGI(TAG, "Jog stopped");
    } else {
      motor_set_speed(handle->motor, speed_percent);
      ESP_LOGI(TAG, "Jog: %.1f%%", speed_percent);
    }
  }
}

void control_manager_set_ema_alpha(control_manager_handle_t handle,
                                   float alpha) {
  if (handle == NULL || handle->loadcell == NULL)
    return;
  hx711_set_ema_alpha(handle->loadcell, alpha);
}

void control_manager_set_filter_size(control_manager_handle_t handle,
                                     uint8_t size) {
  if (handle == NULL || handle->loadcell == NULL)
    return;
  hx711_set_filter_size(handle->loadcell, size);
}

void control_manager_set_cal_offset(control_manager_handle_t handle,
                                    int32_t offset) {
  if (handle == NULL || handle->loadcell == NULL)
    return;
  hx711_calibration_t cal;
  hx711_get_calibration(handle->loadcell, &cal);
  hx711_set_calibration(handle->loadcell, offset, cal.scale);
  ESP_LOGI(TAG, "Cal offset set to %ld", (long)offset);
}

void control_manager_set_cal_scale(control_manager_handle_t handle,
                                   float scale) {
  if (handle == NULL || handle->loadcell == NULL || scale == 0.0f)
    return;
  hx711_calibration_t cal;
  hx711_get_calibration(handle->loadcell, &cal);
  hx711_set_calibration(handle->loadcell, cal.offset, scale);
  ESP_LOGI(TAG, "Cal scale set to %.6f", scale);
}

/*******************************************************************************
 * Private Function Implementations
 ******************************************************************************/

static void control_task(void *arg) {
  control_manager_handle_t mgr = (control_manager_handle_t)arg;
  TickType_t last_wake = xTaskGetTickCount();

  // Subscribe this task to the hardware Task Watchdog Timer
  esp_task_wdt_add(NULL);
  ESP_LOGI(TAG, "Control task started (TWDT armed)");

  while (mgr->running) {
    run_control_loop(mgr);

    // Feed safety watchdog (software)
    safety_feed_watchdog(mgr->safety);
    // Feed hardware Task Watchdog Timer
    esp_task_wdt_reset();

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(mgr->config.control_period_ms));
  }

  // Unsubscribe from TWDT before exit
  esp_task_wdt_delete(NULL);
  ESP_LOGI(TAG, "Control task exiting");
  vTaskDelete(NULL);
}

static void run_control_loop(control_manager_handle_t mgr) {
  // === Read Sensors ===
  encoder_update(mgr->encoder); // Update internal RPM calculation
  float speed_rpm = encoder_get_rpm(mgr->encoder); // Get actual RPM value
  float tension_kg = hx711_update(mgr->loadcell);

  // === Safety Check ===
  safety_readings_t readings = {
      .tension_kg = tension_kg,
      .speed_rpm = speed_rpm,
      .pwm_percent = motor_get_speed(mgr->motor),
      .encoder_active = encoder_is_active(mgr->encoder, 500),
      .loadcell_connected = hx711_is_connected(mgr->loadcell),
      .estop_active = buttons_is_estop_active(mgr->buttons),
  };

  system_state_t state = safety_check(mgr->safety, &readings);

  // === Control Output ===
  float speed_setpoint = 0.0f;
  float new_pwm_output = motor_get_speed(mgr->motor);
  bool update_motor = false;

  xSemaphoreTake(mgr->mutex, portMAX_DELAY);

  if (state == SYSTEM_STATE_RUNNING || state == SYSTEM_STATE_WARNING) {
    switch (mgr->mode) {
    case MODE_AUTO_TENSION:
      // Cascaded control: tension PI outputs speed setpoint
      speed_setpoint =
          pi_compute(&mgr->tension_pi, mgr->tension_setpoint, tension_kg);
      new_pwm_output = pi_compute(&mgr->speed_pi, speed_setpoint, speed_rpm);

      static int log_cnt = 0;
      if (log_cnt++ % 50 == 0) {
        ESP_LOGI(
            TAG,
            "PI RUN: SP=%.2f Act=%.2f | SpdSP=%.2f SpdAct=%.2f -> PWM=%.2f",
            mgr->tension_setpoint, tension_kg, speed_setpoint, speed_rpm,
            new_pwm_output);
      }
      break;

    case MODE_MANUAL:
      // Direct speed control
      speed_setpoint = mgr->speed_setpoint;
      new_pwm_output = pi_compute(&mgr->speed_pi, speed_setpoint, speed_rpm);
      break;

    case MODE_TUNING_SPEED:
      // Speed loop auto-tune: relay output goes directly to PWM
      if (autotune_is_active(mgr->autotuner)) {
        new_pwm_output = autotune_update(mgr->autotuner, 600.0f, speed_rpm);
      }
      break;

    case MODE_TUNING_TENSION:
      // Tension loop auto-tune: relay output is SPEED SETPOINT,
      // routed through speed PI inner loop (must stay active!)
      if (autotune_is_active(mgr->autotuner)) {
        speed_setpoint =
            autotune_update(mgr->autotuner, mgr->tension_setpoint, tension_kg);
        new_pwm_output = pi_compute(&mgr->speed_pi, speed_setpoint, speed_rpm);
      }
      break;

    default:
      break;
    }
    update_motor = true;

    // Check auto-tune completion (for both tuning modes)
    if ((mgr->mode == MODE_TUNING_SPEED || mgr->mode == MODE_TUNING_TENSION) &&
        autotune_is_active(mgr->autotuner) == false &&
        autotune_is_complete(mgr->autotuner)) {
      // Apply results to PI controller (fast, in-memory)
      pi_controller_t *target =
          (mgr->mode == MODE_TUNING_SPEED) ? &mgr->speed_pi : &mgr->tension_pi;
      autotune_apply_result(mgr->autotuner, target);

      // Save tuning mode and gains for deferred NVS write (outside mutex)
      bool was_speed_tune = (mgr->mode == MODE_TUNING_SPEED);
      float saved_kp = target->kp;
      float saved_ki = target->ki;

      mgr->mode = MODE_IDLE;
      lcd_show_message(mgr->lcd, "Tuning complete!", 3000);

      // Release mutex BEFORE NVS writes to avoid blocking control loop
      xSemaphoreGive(mgr->mutex);

      // Deferred NVS writes (can take tens of ms)
      autotune_save_to_nvs(mgr->autotuner, was_speed_tune
                                               ? AUTOTUNE_SPEED_LOOP
                                               : AUTOTUNE_TENSION_LOOP);

      // Also save to web config namespace for UI sync (x10000 precision)
      nvs_handle_t nvs;
      if (nvs_open("config", NVS_READWRITE, &nvs) == ESP_OK) {
        if (was_speed_tune) {
          nvs_set_i32(nvs, "speed_kp", (int32_t)(saved_kp * 10000.0f));
          nvs_set_i32(nvs, "speed_ki", (int32_t)(saved_ki * 10000.0f));
        } else {
          nvs_set_i32(nvs, "tension_kp", (int32_t)(saved_kp * 10000.0f));
          nvs_set_i32(nvs, "tension_ki", (int32_t)(saved_ki * 10000.0f));
        }
        nvs_commit(nvs);
        nvs_close(nvs);
        ESP_LOGI(TAG, "Auto-tune results saved to config: Kp=%.4f Ki=%.4f",
                 saved_kp, saved_ki);
      }

      // Re-take mutex to continue with status update below
      xSemaphoreTake(mgr->mutex, portMAX_DELAY);
    }
  } else if (state == SYSTEM_STATE_FAULT ||
             state == SYSTEM_STATE_EMERGENCY_STOP) {
    // Fault condition - stop immediately
    new_pwm_output = 0.0f;
    update_motor = true;
    pi_set_enabled(&mgr->speed_pi, false);
    pi_set_enabled(&mgr->tension_pi, false);
  }

  // Update status (always read true PWM from motor)
  mgr->status.tension_kg = tension_kg;
  mgr->status.speed_rpm = speed_rpm;
  mgr->status.pwm_percent =
      update_motor ? new_pwm_output : motor_get_speed(mgr->motor);
  mgr->status.tension_setpoint = mgr->tension_setpoint;
  mgr->status.speed_setpoint = speed_setpoint;
  mgr->status.tension_error = mgr->tension_setpoint - tension_kg;
  mgr->status.speed_error = speed_setpoint - speed_rpm;
  mgr->status.mode = mgr->mode;
  mgr->status.system_state = state;
  mgr->status.fault_flags = safety_get_faults(mgr->safety);

  hx711_calibration_t cal;
  hx711_get_calibration(mgr->loadcell, &cal);
  mgr->status.calibrated = cal.valid;
  mgr->status.uptime_seconds =
      (esp_timer_get_time() - mgr->start_time_us) / 1000000;

  xSemaphoreGive(mgr->mutex);

  // === Apply Motor Output ===
  if (update_motor) {
    motor_set_speed(mgr->motor, new_pwm_output);
  }
  // Always update motor state machine (for ramping/soft-start)
  motor_update(mgr->motor);

  // === Log Data ===
  log_sample_t sample = {
      .timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000),
      .tension_setpoint = mgr->tension_setpoint,
      .tension_actual = tension_kg,
      .speed_setpoint = speed_setpoint,
      .speed_actual = speed_rpm,
      .pwm_output = update_motor ? new_pwm_output : motor_get_speed(mgr->motor),
      .tension_error = mgr->tension_setpoint - tension_kg,
      .speed_error = speed_setpoint - speed_rpm,
      .state = state,
      .fault_flags = mgr->status.fault_flags,
  };
  logger_log_sample(mgr->logger, &sample);
}

static void ui_task(void *arg) {
  control_manager_handle_t mgr = (control_manager_handle_t)arg;
  TickType_t last_wake = xTaskGetTickCount();

  ESP_LOGI(TAG, "UI task started");

  while (mgr->running) {
    // Process button input
    buttons_update(mgr->buttons);

    // Process LCD input
    lcd_process_input(mgr->lcd);

    // Update LCD display
    system_status_t status;
    control_manager_get_status(mgr, &status);

    lcd_display_data_t lcd_data = {
        .tension_setpoint = status.tension_setpoint,
        .tension_actual = status.tension_kg,
        .speed_actual = status.speed_rpm,
        .speed_setpoint = status.speed_setpoint,
        .pwm_percent = status.pwm_percent,
        .system_state = status.system_state,
        .fault_flags = status.fault_flags,
        .calibrated = status.calibrated,
        .tuned = status.tuned,
        .uptime_seconds = status.uptime_seconds,
        .speed_kp = mgr->speed_pi.kp,
        .speed_ki = mgr->speed_pi.ki,
        .tension_kp = mgr->tension_pi.kp,
        .tension_ki = mgr->tension_pi.ki,
        .encoder_ppr = encoder_get_ppr(mgr->encoder),
        .max_rpm = CONFIG_MOTOR_MAX_RPM,
        .tension_step = mgr->tension_step,
    };
    lcd_update(mgr->lcd, &lcd_data);

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(mgr->config.ui_period_ms));
  }

  ESP_LOGI(TAG, "UI task exiting");
  vTaskDelete(NULL);
}

static void on_button_event(button_event_t event, void *user_data) {
  control_manager_handle_t mgr = (control_manager_handle_t)user_data;

  switch (event) {
  case BTN_EVENT_RUN_PRESSED:
    control_manager_run(mgr);
    break;

  case BTN_EVENT_STOP_PRESSED:
    control_manager_stop_controlled(mgr);
    break;

  case BTN_EVENT_STOP_LONG_PRESS:
    // Long press on stop = reset faults
    control_manager_reset_faults(mgr);
    break;

  case BTN_EVENT_ESTOP_ACTIVATED:
    control_manager_emergency_stop(mgr);
    break;

  default:
    break;
  }
}

static void on_lcd_command(uint8_t command, float value, void *user_data) {
  control_manager_handle_t mgr = (control_manager_handle_t)user_data;

  switch (command) {
  case 'R':
  case 'r':
    control_manager_run(mgr);
    break;

  case 'S':
  case 's':
    control_manager_stop_controlled(mgr);
    break;

  case 'T':
  case 't':
    control_manager_start_autotune(mgr, value == 0); // Default to speed loop
    break;

  case 'C':
  case 'c':
    if (value > 0) {
      control_manager_start_calibration(mgr, value);
    } else {
      control_manager_start_tare(mgr);
    }
    break;

  case '+':
    control_manager_set_tension(mgr, mgr->tension_setpoint + mgr->tension_step);
    break;

  case '-':
    control_manager_set_tension(mgr, mgr->tension_setpoint - mgr->tension_step);
    break;

  default:
    break;
  }
}

/**
 * @brief Handle structured commands from LCD menu system
 */
static void on_menu_command(lcd_command_id_t cmd, float value,
                            void *user_data) {
  control_manager_handle_t mgr = (control_manager_handle_t)user_data;

  switch (cmd) {
  case LCD_CMD_TENSION_UP:
    control_manager_set_tension(mgr, mgr->tension_setpoint + value);
    break;
  case LCD_CMD_TENSION_DOWN:
    control_manager_set_tension(mgr, mgr->tension_setpoint - value);
    break;
  case LCD_CMD_JOG_LEFT:
    control_manager_jog(mgr, -30.0f); /* 30% speed left */
    break;
  case LCD_CMD_JOG_RIGHT:
    control_manager_jog(mgr, 30.0f); /* 30% speed right */
    break;
  case LCD_CMD_JOG_STOP:
    control_manager_jog(mgr, 0.0f);
    break;
  case LCD_CMD_TARE:
    control_manager_start_tare(mgr);
    break;
  case LCD_CMD_CALIBRATE:
    control_manager_start_calibration(mgr, value);
    break;
  case LCD_CMD_AUTOTUNE_SPEED:
    control_manager_start_autotune(mgr, true);
    break;
  case LCD_CMD_AUTOTUNE_TENSION:
    control_manager_start_autotune(mgr, false);
    break;
  case LCD_CMD_CLEAR_FAULTS:
    control_manager_reset_faults(mgr);
    break;
  case LCD_CMD_SET_TENSION_STEP:
    mgr->tension_step = value;
    ESP_LOGI(TAG, "Tension step set to %.1f kg", value);
    break;
  case LCD_CMD_SET_PPR: {
    uint16_t ppr = (uint16_t)value;
    encoder_set_ppr(mgr->encoder, ppr);
    /* Save to NVS */
    nvs_handle_t nvs;
    if (nvs_open("config", NVS_READWRITE, &nvs) == ESP_OK) {
      nvs_set_u16(nvs, "ppr", ppr);
      nvs_commit(nvs);
      nvs_close(nvs);
    }
    ESP_LOGI(TAG, "PPR set to %u", ppr);
    break;
  }
  case LCD_CMD_SET_SPEED_KP:
    pi_set_gains(&mgr->speed_pi, value, mgr->speed_pi.ki);
    ESP_LOGI(TAG, "Speed Kp set to %.4f", value);
    break;
  case LCD_CMD_SET_SPEED_KI:
    pi_set_gains(&mgr->speed_pi, mgr->speed_pi.kp, value);
    ESP_LOGI(TAG, "Speed Ki set to %.4f", value);
    break;
  case LCD_CMD_SET_TENSION_KP:
    pi_set_gains(&mgr->tension_pi, value, mgr->tension_pi.ki);
    ESP_LOGI(TAG, "Tension Kp set to %.4f", value);
    break;
  case LCD_CMD_SET_TENSION_KI:
    pi_set_gains(&mgr->tension_pi, mgr->tension_pi.kp, value);
    ESP_LOGI(TAG, "Tension Ki set to %.4f", value);
    break;
  default:
    break;
  }
}
