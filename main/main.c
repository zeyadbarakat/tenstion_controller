/**
 * @file main.c
 * @brief Tension Control System - Main Application Entry Point
 *
 * ESP32-S3 Tension Controller for Unwinding Applications
 * ======================================================
 *
 * System: Motor → Gearbox → Wire Roll ~~wire~~ Loadcell ~~wire~~ Machine
 * The machine pulls wire creating tension; the motor unwinds the roll
 * to release wire and reduce tension (inverse plant relationship).
 *
 * Features:
 * - Cascaded PI control (outer tension loop, inner speed loop)
 * - Inverted output for unwinder (more speed = less tension)
 * - Relay feedback auto-tuning (no-overshoot rule, Kp = 0.20 × Ku)
 * - Load cell with NVS calibration storage
 * - PCNT quadrature encoder for speed measurement
 * - PWM motor control with soft-start
 * - Safety monitoring with configurable limits (web UI + NVS)
 * - Live hot-reload of PI gains and safety parameters
 * - Web interface with Dashboard, Tuning, Calibration, Safety tabs
 * - WebSocket real-time updates at 50Hz
 * - RGB LED status indicator
 *
 * Hardware:
 * - ESP32-S3 MCU
 * - 600 PPR quadrature encoder
 * - HX711 24-bit load cell ADC
 * - DC motor with H-bridge driver (unidirectional unwinding)
 * - Heavy gearbox (provides natural braking when motor stops)
 * - RUN/STOP/E-STOP buttons
 *
 * @copyright 2026
 */

#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <stdio.h>
#include <string.h>

#include "control_manager.h"
#include "lcd_menu.h"

#include "safety.h"
#include "system_config.h"
#include "web_server.h"
#include "wifi_manager.h"

static const char *TAG = "main";

#define FLASH_BUTTON_GPIO 0

static void init_flash_button(void) {
  gpio_config_t io_conf;
  io_conf.pin_bit_mask = (1ULL << FLASH_BUTTON_GPIO);
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&io_conf);
  ESP_LOGI(TAG, "Flash button initialized on GPIO %d", FLASH_BUTTON_GPIO);
}

// Web status callback handles (Must be above status_led_task)
static control_manager_handle_t g_ctrl_mgr = NULL;
static web_server_handle_t g_web_srv = NULL;
bool g_wifi_enabled = false;

// Background task to blink/drive the discrete status LEDs
static void status_led_task(void *param) {
  TickType_t last_wake = xTaskGetTickCount();

  // Configure discrete LEDs as outputs
  gpio_config_t led_conf = {.pin_bit_mask = (1ULL << PIN_LED_RUNNING) |
                                            (1ULL << PIN_LED_FAULT),
                            .mode = GPIO_MODE_OUTPUT,
                            .pull_up_en = GPIO_PULLUP_DISABLE,
                            .pull_down_en = GPIO_PULLDOWN_DISABLE,
                            .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&led_conf);

  // Start LEDs off
  gpio_set_level(PIN_LED_RUNNING, 0);
  gpio_set_level(PIN_LED_FAULT, 0);

  bool toggle = false;

  while (1) {
    toggle = !toggle;

    if (g_ctrl_mgr) {
      system_status_t sys_status;
      control_manager_get_status(g_ctrl_mgr, &sys_status);

      // 1. FAULT LED Logic (RED)
      if (sys_status.system_state == SYSTEM_STATE_FAULT ||
          sys_status.system_state == SYSTEM_STATE_EMERGENCY_STOP) {
        // Blink fault LED rapidly
        gpio_set_level(PIN_LED_FAULT, toggle);
      } else if (sys_status.system_state == SYSTEM_STATE_WARNING) {
        // Warning = Solid Red/Yellow
        gpio_set_level(PIN_LED_FAULT, 1);
      } else {
        // No faults
        gpio_set_level(PIN_LED_FAULT, 0);
      }

      // 2. RUN LED Logic (GREEN)
      if (sys_status.system_state == SYSTEM_STATE_STARTING ||
          sys_status.system_state == SYSTEM_STATE_STOPPING) {
        // Transitioning = blinking green
        gpio_set_level(PIN_LED_RUNNING, toggle);
      } else if (sys_status.system_state == SYSTEM_STATE_RUNNING ||
                 sys_status.mode == 3 /* MODE_TUNING_SPEED */ ||
                 sys_status.mode == 4 /* MODE_TUNING_TENSION */) {
        // Running active = solid green
        gpio_set_level(PIN_LED_RUNNING, 1);
      } else {
        // IDLE = off
        gpio_set_level(PIN_LED_RUNNING, 0);
      }
    }

    // 2.5Hz blink rate (200ms toggle = 400ms period)
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(200));
  }
}

// WiFi event handler
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT) {
    switch (event_id) {
    case WIFI_EVENT_STA_START:
      ESP_LOGI(TAG, "WiFi STA started, connecting...");
      esp_wifi_connect();
      break;
    case WIFI_EVENT_STA_DISCONNECTED:
      ESP_LOGW(TAG, "WiFi disconnected, reconnecting...");
      esp_wifi_connect();
      break;
    case WIFI_EVENT_AP_START:
      ESP_LOGI(TAG, "WiFi AP started");
      break;
    case WIFI_EVENT_AP_STACONNECTED:
      ESP_LOGI(TAG, "Client connected to AP");
      break;
    }
  } else if (event_base == IP_EVENT) {
    if (event_id == IP_EVENT_STA_GOT_IP) {
      ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
      ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }
  }
}

static esp_err_t init_wifi_ap(void) {
  ESP_LOGI(TAG, "Initializing WiFi in AP mode...");

  // Initialize TCP/IP stack
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_ap();

  // Initialize WiFi
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  // Register event handlers
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

  // Configure AP using Kconfig values
  wifi_config_t wifi_config = {
      .ap =
          {
              .ssid = WIFI_AP_SSID,
              .ssid_len = strlen(WIFI_AP_SSID),
              .password = WIFI_AP_PASSWORD,
              .channel = 1,
              .max_connection = 4,
              .authmode = WIFI_AUTH_WPA2_PSK,
          },
  };

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "WiFi AP started - SSID: %s", WIFI_AP_SSID);
  ESP_LOGI(TAG, "Connect to http://192.168.4.1 for web interface");

  g_wifi_enabled = true;
  return ESP_OK;
}

// WebSocket broadcast task - pushes sensor data at 50Hz
static void ws_broadcast_task(void *param) {
  char json_buf[320];
  TickType_t last_wake = xTaskGetTickCount();

  ESP_LOGI(TAG, "WebSocket broadcast task started (50Hz)");

  while (1) {
    // Only broadcast if we have clients and control manager is ready
    if (g_web_srv && g_ctrl_mgr && web_server_ws_client_count(g_web_srv) > 0) {
      system_status_t status;
      control_manager_get_status(g_ctrl_mgr, &status);

      // Build compact JSON for WebSocket
      snprintf(
          json_buf, sizeof(json_buf),
          "{\"tension\":%.3f,\"tensionSP\":%.3f,\"speed\":%.0f,\"speedSP\":%."
          "0f,"
          "\"pwm\":%.1f,\"state\":%d,\"mode\":%d,\"faults\":%d,\"cal\":%d,"
          "\"uptime\":%lu,\"rawAdc\":%ld}",
          status.tension_kg, status.tension_setpoint, status.speed_rpm,
          status.speed_setpoint, status.pwm_percent, status.system_state,
          status.mode, status.fault_flags, status.calibrated ? 1 : 0,
          (unsigned long)status.uptime_seconds, (long)status.raw_adc);

      // Broadcast to all WebSocket clients
      web_server_ws_broadcast(g_web_srv, json_buf);
    }

    // Run at 50Hz (20ms period) - lower priority than control loop
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));
  }
}

// Removed duplicates

static void web_status_cb(web_status_data_t *status, void *user_data) {
  if (g_ctrl_mgr && status) {
    system_status_t sys_status;
    control_manager_get_status(g_ctrl_mgr, &sys_status);

    status->tension_kg = sys_status.tension_kg;
    status->tension_setpoint = sys_status.tension_setpoint;
    status->speed_rpm = sys_status.speed_rpm;
    status->speed_setpoint = sys_status.speed_setpoint;
    status->pwm_percent = sys_status.pwm_percent;
    status->system_state = sys_status.system_state;
    status->fault_flags = sys_status.fault_flags;
    status->calibrated = sys_status.calibrated;
    status->uptime_seconds = sys_status.uptime_seconds;
    status->detected_rpm = sys_status.detected_rpm;
  }
}

static void web_command_cb(web_command_t cmd, float value, void *user_data) {
  ESP_LOGI(TAG, "Web command: %d, value: %.2f", cmd, value);

  switch (cmd) {
  case WEB_CMD_START:
    control_manager_run(g_ctrl_mgr);
    break;
  case WEB_CMD_STOP:
    control_manager_stop_controlled(g_ctrl_mgr);
    break;
  case WEB_CMD_ESTOP:
    control_manager_emergency_stop(g_ctrl_mgr);
    break;
  case WEB_CMD_SET_TENSION:
    control_manager_set_tension(g_ctrl_mgr, value);
    break;
  case WEB_CMD_TARE:
    control_manager_start_tare(g_ctrl_mgr);
    break;
  case WEB_CMD_CALIBRATE:
    control_manager_start_calibration(g_ctrl_mgr, value);
    break;
  case WEB_CMD_AUTOTUNE_SPEED:
    control_manager_start_autotune(g_ctrl_mgr, true);
    break;
  case WEB_CMD_AUTOTUNE_TENSION:
    control_manager_start_autotune(g_ctrl_mgr, false);
    break;
  case WEB_CMD_RESET_FAULTS:
    control_manager_reset_faults(g_ctrl_mgr);
    break;
  case WEB_CMD_JOG_START: {
    float speed = safety_get_jog_speed_from_nvs();
    ESP_LOGI(TAG, "Jog start: direction %.0f at %.1f%%", value, speed);
    control_manager_jog(g_ctrl_mgr, value > 0 ? speed : -speed);
    break;
  }
  case WEB_CMD_JOG_STOP:
    ESP_LOGI(TAG, "Jog stop");
    control_manager_jog(g_ctrl_mgr, 0.0f);
    break;
  case WEB_CMD_SET_EMA_ALPHA:
    // value is alpha * 100 (e.g., 10 = 0.10)
    control_manager_set_ema_alpha(g_ctrl_mgr, value / 100.0f);
    break;
  case WEB_CMD_SET_MA_WINDOW:
    control_manager_set_filter_size(g_ctrl_mgr, (uint8_t)value);
    break;
  case WEB_CMD_SET_MEDIAN_WINDOW:
    control_manager_set_median_size(g_ctrl_mgr, (uint8_t)value);
    break;
  case WEB_CMD_SET_CAL_OFFSET:
    control_manager_set_cal_offset(g_ctrl_mgr, (int32_t)value);
    break;
  case WEB_CMD_SET_CAL_SCALE:
    control_manager_set_cal_scale(g_ctrl_mgr, value);
    break;
  case WEB_CMD_DETECT_RPM:
    control_manager_detect_rpm(g_ctrl_mgr);
    break;
  case WEB_CMD_SET_PI_GAINS: {
    // Read freshly-saved gains from NVS and apply to live PI controllers
    nvs_handle_t nvs;
    if (nvs_open("config", NVS_READONLY, &nvs) == ESP_OK) {
      int32_t skp = 5000, ski = 1000, tkp = 2000, tki = 500;
      nvs_get_i32(nvs, "speed_kp", &skp);
      nvs_get_i32(nvs, "speed_ki", &ski);
      nvs_get_i32(nvs, "tension_kp", &tkp);
      nvs_get_i32(nvs, "tension_ki", &tki);
      nvs_close(nvs);
      control_manager_set_pi_gains(g_ctrl_mgr, skp / 10000.0f, ski / 10000.0f,
                                   tkp / 10000.0f, tki / 10000.0f);
    }
    break;
  }
  case WEB_CMD_SET_SAFETY: {
    // Read freshly-saved safety limits and apply live
    safety_limits_t limits = safety_get_default_limits();
    nvs_handle_t nvs;
    if (nvs_open("safety", NVS_READONLY, &nvs) == ESP_OK) {
      size_t size = sizeof(safety_limits_t);
      nvs_get_blob(nvs, "limits", &limits, &size);
      nvs_close(nvs);
    }
    control_manager_set_safety_limits(g_ctrl_mgr, &limits);
    break;
  }
  case WEB_CMD_SET_PPR:
    control_manager_set_ppr(g_ctrl_mgr, (uint16_t)value);
    break;
  case WEB_CMD_SET_TENSION_UNIT:
    control_manager_set_tension_unit(g_ctrl_mgr, (uint8_t)value);
    break;
  case WEB_CMD_SET_MAX_TENSION:
    control_manager_set_max_tension(g_ctrl_mgr, value);
    break;
  default:
    break;
  }
}

/*******************************************************************************
 * Application Entry Point
 ******************************************************************************/

void app_main(void) {
  ESP_LOGI(TAG, "=========================================");
  ESP_LOGI(TAG, "   Tension Control System v1.0.0");
  ESP_LOGI(TAG, "   ESP32-S3 Industrial Controller");
  ESP_LOGI(TAG, "=========================================");
  ESP_LOGI(TAG, "");

  // === Initialize Flash Button ===
  init_flash_button();

  // === Initialize NVS ===
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGW(TAG, "NVS partition was truncated and needs to be erased");
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
  ESP_LOGI(TAG, "NVS initialized");

  // Load saved WiFi state from NVS (default: enabled)
  g_wifi_enabled = true;
  {
    nvs_handle_t nvs;
    if (nvs_open("config", NVS_READONLY, &nvs) == ESP_OK) {
      uint8_t saved_wifi = 1;
      if (nvs_get_u8(nvs, "wifi_enabled", &saved_wifi) == ESP_OK) {
        g_wifi_enabled = saved_wifi ? true : false;
        ESP_LOGI(TAG, "Loaded WiFi state from NVS: %s",
                 g_wifi_enabled ? "ENABLED" : "DISABLED");
      }
      nvs_close(nvs);
    }
  }

  // === Initialize WiFi ===
  ret = init_wifi_ap();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "WiFi init failed: %s", esp_err_to_name(ret));
  } else if (!g_wifi_enabled) {
    // If WiFi was disabled last time, stop it after init
    esp_wifi_stop();
    ESP_LOGI(TAG, "WiFi started but immediately stopped (user preference)");
  }

  // === Initialize Web Server ===
  web_server_handle_t web_srv = NULL;
  ret = web_server_init(&web_srv, 80);
  if (ret == ESP_OK) {
    g_web_srv = web_srv; // Store globally for broadcast task
    web_server_set_status_callback(web_srv, web_status_cb, NULL);
    web_server_set_command_callback(web_srv, web_command_cb, NULL);
    web_server_start(web_srv);
    ESP_LOGI(TAG, "Web server started on port 80 (WebSocket enabled)");

    // Start WebSocket broadcast task (priority 3, lower than control loop)
    xTaskCreate(ws_broadcast_task, "ws_broadcast", 4096, NULL, 3, NULL);
  }

  // === Print Configuration ===
  ESP_LOGI(TAG, "Configuration:");
  ESP_LOGI(TAG, "  Encoder: GPIO A=%d, B=%d, PPR=%d", PIN_ENCODER_A,
           PIN_ENCODER_B, ENCODER_PPR);
  ESP_LOGI(TAG, "  HX711:   GPIO DATA=%d, CLK=%d", PIN_HX711_DATA,
           PIN_HX711_CLOCK);
  ESP_LOGI(TAG, "  Motor:   GPIO RPWM=%d, LPWM=%d, EN=%d/%d, Freq=%d Hz",
           PIN_MOTOR_RPWM, PIN_MOTOR_LPWM, PIN_MOTOR_R_EN, PIN_MOTOR_L_EN,
           MOTOR_PWM_FREQ_HZ);
  ESP_LOGI(TAG, "  Buttons: RUN=%d, STOP=%d, ESTOP=%d", PIN_BTN_RUN,
           PIN_BTN_STOP, PIN_BTN_ESTOP);
  ESP_LOGI(TAG, "");

  // === Initialize Control Manager ===
  control_manager_config_t config = control_manager_get_default_config();

  // Apply GPIO configuration
  config.encoder_a_gpio = PIN_ENCODER_A;
  config.encoder_b_gpio = PIN_ENCODER_B;
  config.hx711_data_gpio = PIN_HX711_DATA;
  config.hx711_clock_gpio = PIN_HX711_CLOCK;
  config.motor_rpwm_gpio = PIN_MOTOR_RPWM;
  config.motor_lpwm_gpio = PIN_MOTOR_LPWM;
  config.motor_r_en_gpio = PIN_MOTOR_R_EN;
  config.motor_l_en_gpio = PIN_MOTOR_L_EN;
  config.button_run_gpio = PIN_BTN_RUN;
  config.button_stop_gpio = PIN_BTN_STOP;
  config.button_estop_gpio = PIN_BTN_ESTOP;

  // Control parameters
  config.control_period_ms = (1000 / TENSION_LOOP_FREQ_HZ);
  config.ui_period_ms = 200;              // 5Hz UI update
  config.default_tension_setpoint = 5.0f; // Default 5kg

  ret = control_manager_init(&g_ctrl_mgr, &config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize control manager: %s",
             esp_err_to_name(ret));
    ESP_LOGE(TAG, "System halted - check hardware connections");
    while (1) {
      // Blink fault LED to signal error
      gpio_set_level(PIN_LED_FAULT, 1);
      vTaskDelay(pdMS_TO_TICKS(200));
      gpio_set_level(PIN_LED_FAULT, 0);
      vTaskDelay(pdMS_TO_TICKS(800));
    }
  }

  ESP_LOGI(TAG, "Control manager initialized successfully");

  // === Start discrete status LEDs (RUNNING + FAULT) ===
  xTaskCreate(status_led_task, "status_leds", 2048, NULL, 3, NULL);
  ESP_LOGI(TAG, "Status LED task started (GPIO %d, %d)", PIN_LED_RUNNING,
           PIN_LED_FAULT);
  ESP_LOGI(TAG, "");

  // === Start Control System ===
  ret = control_manager_start(g_ctrl_mgr);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start control system: %s", esp_err_to_name(ret));
  }

  ESP_LOGI(TAG, "=========================================");
  ESP_LOGI(TAG, "   System Ready - Waiting for RUN");
  ESP_LOGI(TAG, "=========================================");
  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "Web Interface: http://192.168.4.1");
  ESP_LOGI(TAG, "WiFi: SSID=%s", WIFI_AP_SSID);
  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "UART Commands:");
  ESP_LOGI(TAG, "  R     - Run/Start system");
  ESP_LOGI(TAG, "  S     - Stop system");
  ESP_LOGI(TAG, "  T     - Start auto-tune");
  ESP_LOGI(TAG, "  C     - Tare calibration");
  ESP_LOGI(TAG, "  C<kg> - Span calibration (e.g., C5.0)");
  ESP_LOGI(TAG, "  +/-   - Adjust tension setpoint");
  ESP_LOGI(TAG, "");

  // === Main Loop ===
  uint32_t loop_count = 0;

  // g_wifi_enabled is global (loaded from NVS above)
  static uint32_t btn_press_time = 0;

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(100)); // 100ms interval for faster updates

    loop_count++;

    // Get current status
    system_status_t status;
    control_manager_get_status(g_ctrl_mgr, &status);

    if (gpio_get_level(FLASH_BUTTON_GPIO) == 0) {
      if (btn_press_time == 0) {
        btn_press_time = loop_count;
      } else if (loop_count - btn_press_time >=
                 10) { // ~1 second hold (10 * 100ms)
        g_wifi_enabled = !g_wifi_enabled;
        if (g_wifi_enabled) {
          esp_wifi_start();
          ESP_LOGI(TAG, "WiFi ENABLED (boot button held)");
        } else {
          esp_wifi_stop();
          ESP_LOGI(TAG, "WiFi DISABLED (boot button held)");
        }
        // Save WiFi state to NVS
        nvs_handle_t nvs;
        if (nvs_open("config", NVS_READWRITE, &nvs) == ESP_OK) {
          nvs_set_u8(nvs, "wifi_enabled", g_wifi_enabled ? 1 : 0);
          nvs_commit(nvs);
          nvs_close(nvs);
          ESP_LOGI(TAG, "WiFi state saved to NVS");
        }
        btn_press_time = 0;
        vTaskDelay(pdMS_TO_TICKS(1000)); // Debounce
      }
    } else {
      btn_press_time = 0;
    }

    // Log status every 30 loops (3 seconds)
    if (loop_count % 30 == 0) {
      ESP_LOGI(TAG,
               "[%lu] Status: State=%d, Tension=%.2f/%.2f kg, Speed=%.0f rpm, "
               "PWM=%.1f%%",
               (unsigned long)loop_count, status.system_state,
               status.tension_kg, status.tension_setpoint, status.speed_rpm,
               status.pwm_percent);

      if (status.fault_flags != 0) {
        ESP_LOGW(TAG, "Active faults: 0x%04X", status.fault_flags);
      }

      ESP_LOGI(TAG, "Free heap: %lu bytes",
               (unsigned long)esp_get_free_heap_size());
    }
  }
}
