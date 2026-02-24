/**
 * @file main.c
 * @brief Tension Control System - Main Application Entry Point
 *
 * ESP32-S3 Tension Controller for Unwinding Applications
 * ======================================================
 *
 * Features:
 * - Cascaded PI control (outer tension loop, inner speed loop)
 * - Relay feedback auto-tuning (Ziegler-Nichols)
 * - Load cell with NVS calibration storage
 * - PCNT quadrature encoder for speed measurement
 * - PWM motor control with soft-start
 * - Safety monitoring with E-STOP support
 * - Web interface for monitoring and control
 * - RGB LED status indicator
 *
 * Hardware:
 * - ESP32-S3 MCU
 * - 600 PPR quadrature encoder
 * - HX711 24-bit load cell ADC
 * - DC motor with H-bridge driver
 * - RUN/STOP/E-STOP buttons
 *
 * @copyright 2024
 */

#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <stdio.h>
#include <string.h>

#include "control_manager.h"
#include "led_strip.h"
#include "system_config.h"
#include "web_server.h"
#include "wifi_manager.h"

static const char *TAG = "main";

// Status LED config - WS2812 on GPIO38 (ESP32-S3-DevKitC-1 v1.1)
// Use GPIO48 for older ESP32-S3-DevKitC-1 boards
#define RGB_LED_GPIO 38
#define FLASH_BUTTON_GPIO 0

// WS2812 LED strip handle (using RMT)
static led_strip_handle_t led_strip = NULL;

static void led_set_color(uint8_t r, uint8_t g, uint8_t b) {
  if (led_strip) {
    led_strip_set_pixel(led_strip, 0, r, g, b);
    led_strip_refresh(led_strip);
  }
}

static void init_status_led(void) {
  // Configure WS2812 LED using RMT
  led_strip_config_t strip_config = {
      .strip_gpio_num = RGB_LED_GPIO,
      .max_leds = 1,
  };
  led_strip_rmt_config_t rmt_config = {
      .resolution_hz = 10 * 1000 * 1000, // 10MHz
      .flags.with_dma = false,
  };
  ESP_ERROR_CHECK(
      led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
  led_strip_clear(led_strip);
  ESP_LOGI(TAG, "WS2812 Status LED initialized on GPIO %d", RGB_LED_GPIO);
}

static void init_flash_button(void) {
  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << FLASH_BUTTON_GPIO),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&io_conf);
  ESP_LOGI(TAG, "Flash button initialized on GPIO %d", FLASH_BUTTON_GPIO);
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

  // Configure AP
  wifi_config_t wifi_config = {
      .ap =
          {
              .ssid = "TensionCTRL",
              .ssid_len = strlen("TensionCTRL"),
              .password = "tension123",
              .channel = 1,
              .max_connection = 4,
              .authmode = WIFI_AUTH_WPA2_PSK,
          },
  };

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "WiFi AP started - SSID: TensionCTRL, Password: tension123");
  ESP_LOGI(TAG, "Connect to http://192.168.4.1 for web interface");

  return ESP_OK;
}

// Web status callback
static control_manager_handle_t g_ctrl_mgr = NULL;
static web_server_handle_t g_web_srv = NULL;

// WebSocket broadcast task - pushes sensor data at 50Hz
static void ws_broadcast_task(void *param) {
  char json_buf[256];
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
          "{\"tension\":%.2f,\"tensionSP\":%.1f,\"speed\":%.0f,\"speedSP\":%."
          "0f,"
          "\"pwm\":%.1f,\"state\":%d,\"faults\":%d,\"cal\":%d,\"uptime\":%lu}",
          status.tension_kg, status.tension_setpoint, status.speed_rpm,
          status.speed_setpoint, status.pwm_percent, status.system_state,
          status.fault_flags, status.calibrated ? 1 : 0,
          (unsigned long)status.uptime_seconds);

      // Broadcast to all WebSocket clients
      web_server_ws_broadcast(g_web_srv, json_buf);
    }

    // Run at 50Hz (20ms period) - lower priority than control loop
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));
  }
}

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
  case WEB_CMD_JOG_START:
    // Jog motor at 25% speed when stopped
    // value: -1 = left/reverse, +1 = right/forward
    ESP_LOGI(TAG, "Jog start: direction %.0f", value);
    control_manager_jog(g_ctrl_mgr, value > 0 ? 25.0f : -25.0f);
    break;
  case WEB_CMD_JOG_STOP:
    ESP_LOGI(TAG, "Jog stop");
    control_manager_jog(g_ctrl_mgr, 0.0f);
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

  // === Initialize Status LED ===
  init_status_led();
  init_flash_button();

  // Blink LED to show startup
  for (int i = 0; i < 3; i++) {
    gpio_set_level(RGB_LED_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(RGB_LED_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
  }

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
  static bool wifi_enabled = true;
  {
    nvs_handle_t nvs;
    if (nvs_open("config", NVS_READONLY, &nvs) == ESP_OK) {
      uint8_t saved_wifi = 1;
      if (nvs_get_u8(nvs, "wifi_enabled", &saved_wifi) == ESP_OK) {
        wifi_enabled = saved_wifi ? true : false;
        ESP_LOGI(TAG, "Loaded WiFi state from NVS: %s",
                 wifi_enabled ? "ENABLED" : "DISABLED");
      }
      nvs_close(nvs);
    }
  }

  // === Initialize WiFi ===
  ret = init_wifi_ap();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "WiFi init failed: %s", esp_err_to_name(ret));
  } else if (!wifi_enabled) {
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
  ESP_LOGI(TAG, "  Motor:   GPIO PWM=%d, DIR=%d, Freq=%d Hz", PIN_MOTOR_PWM,
           PIN_MOTOR_DIR, MOTOR_PWM_FREQ_HZ);
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
  config.motor_pwm_gpio = PIN_MOTOR_PWM;
  config.motor_dir_gpio = PIN_MOTOR_DIR;
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
      // Blink error pattern
      gpio_set_level(RGB_LED_GPIO, 1);
      vTaskDelay(pdMS_TO_TICKS(200));
      gpio_set_level(RGB_LED_GPIO, 0);
      vTaskDelay(pdMS_TO_TICKS(800));
    }
  }

  ESP_LOGI(TAG, "Control manager initialized successfully");
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
  ESP_LOGI(TAG, "WiFi: SSID=TensionCTRL, Password=tension123");
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

  // wifi_enabled is now static global (loaded from NVS above)
  static uint32_t btn_press_time = 0;

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(500)); // 500ms interval for LED update

    loop_count++;

    // Get current status
    system_status_t status;
    control_manager_get_status(g_ctrl_mgr, &status);

    // LED pattern: 3 status flashes + 1 WiFi indicator flash
    // Pattern cycles: [Status ON, Status OFF, Status ON, Status OFF, Status ON,
    // Status OFF, WiFi ON, WiFi OFF] Using loop_count with 500ms delay = 4
    // second full cycle
    static uint8_t led_phase = 0;
    led_phase = (led_phase + 1) % 8; // 8 phases for the pattern

    // GRB color order for WS2812
    uint8_t g = 0, r = 0, b = 0;

    if (led_phase < 6) {
      // Phases 0-5: Status color (3 blinks = on/off/on/off/on/off)
      bool on = (led_phase % 2 == 0); // Even phases = LED on
      if (status.fault_flags != 0) {
        // Fault - red
        r = on ? 40 : 0;
      } else {
        // Normal - green
        g = on ? 20 : 0;
      }
    } else {
      // Phases 6-7: WiFi indicator (1 blink)
      bool on = (led_phase == 6);
      if (wifi_enabled) {
        // WiFi on - blue
        b = on ? 30 : 0;
      }
      // WiFi off - no light (just stays off)
    }

    led_set_color(g, r, b);

    // Check flash button for WiFi toggle (GPIO0)
    if (gpio_get_level(FLASH_BUTTON_GPIO) == 0) {
      if (btn_press_time == 0) {
        btn_press_time = loop_count;
      } else if (loop_count - btn_press_time >= 2) { // ~1 second hold
        wifi_enabled = !wifi_enabled;
        if (wifi_enabled) {
          esp_wifi_start();
          ESP_LOGI(TAG, "WiFi ENABLED (boot button held)");
        } else {
          esp_wifi_stop();
          ESP_LOGI(TAG, "WiFi DISABLED (boot button held)");
        }
        // Save WiFi state to NVS
        nvs_handle_t nvs;
        if (nvs_open("config", NVS_READWRITE, &nvs) == ESP_OK) {
          nvs_set_u8(nvs, "wifi_enabled", wifi_enabled ? 1 : 0);
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

    // Log status every 10 loops (5 seconds)
    if (loop_count % 10 == 0) {
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
