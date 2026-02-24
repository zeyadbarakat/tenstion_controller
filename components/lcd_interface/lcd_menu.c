/**
 * @file lcd_menu.c
 * @brief UART Terminal Interface with ANSI Escape Codes
 */

#include "lcd_menu.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>


static const char *TAG = "lcd";

#define UART_BUF_SIZE 256

// ANSI escape codes
#define ANSI_CLEAR "\033[2J"
#define ANSI_HOME "\033[H"
#define ANSI_BOLD "\033[1m"
#define ANSI_RESET "\033[0m"
#define ANSI_RED "\033[31m"
#define ANSI_GREEN "\033[32m"
#define ANSI_YELLOW "\033[33m"
#define ANSI_BLUE "\033[34m"
#define ANSI_CYAN "\033[36m"

struct lcd_menu_s {
  int uart_num;
  menu_screen_t current_screen;
  lcd_command_callback_t callback;
  void *callback_user_data;
  lcd_display_data_t last_data;
  char input_buffer[32];
  uint8_t input_pos;
  char message[64];
  uint32_t message_expire;
};

static void lcd_tx(lcd_handle_t handle, const char *str);

esp_err_t lcd_init(lcd_handle_t *handle, int uart_num) {
  if (handle == NULL)
    return ESP_ERR_INVALID_ARG;

  struct lcd_menu_s *lcd = calloc(1, sizeof(struct lcd_menu_s));
  if (lcd == NULL)
    return ESP_ERR_NO_MEM;

  lcd->uart_num = uart_num;
  lcd->current_screen = MENU_MAIN;

  // Configure UART
  uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };

  esp_err_t ret =
      uart_driver_install(uart_num, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
  if (ret != ESP_OK) {
    free(lcd);
    return ret;
  }

  uart_param_config(uart_num, &uart_config);

  // Clear screen
  lcd_tx(lcd, ANSI_CLEAR ANSI_HOME);

  ESP_LOGI(TAG, "LCD interface initialized on UART%d", uart_num);

  *handle = lcd;
  return ESP_OK;
}

void lcd_deinit(lcd_handle_t handle) {
  if (handle) {
    uart_driver_delete(handle->uart_num);
    free(handle);
  }
}

static void lcd_tx(lcd_handle_t handle, const char *str) {
  uart_write_bytes(handle->uart_num, str, strlen(str));
}

static const char *state_to_string(uint8_t state) {
  switch (state) {
  case 0:
    return "IDLE";
  case 1:
    return "STARTING";
  case 2:
    return "RUNNING";
  case 3:
    return "STOPPING";
  case 4:
    return "WARNING";
  case 5:
    return "FAULT";
  case 6:
    return "E-STOP";
  default:
    return "???";
  }
}

void lcd_update(lcd_handle_t handle, const lcd_display_data_t *data) {
  if (handle == NULL || data == NULL)
    return;

  memcpy(&handle->last_data, data, sizeof(lcd_display_data_t));

  char line[128];

  // Home cursor
  lcd_tx(handle, ANSI_HOME);

  // Header
  lcd_tx(handle, ANSI_BOLD ANSI_CYAN "=== TENSION CONTROL SYSTEM ===" ANSI_RESET
                                     "\r\n\r\n");

  // State with color
  const char *color = ANSI_GREEN;
  if (data->system_state >= 4)
    color = ANSI_YELLOW;
  if (data->system_state >= 5)
    color = ANSI_RED;

  snprintf(line, sizeof(line), "State: %s%s" ANSI_RESET "     \r\n", color,
           state_to_string(data->system_state));
  lcd_tx(handle, line);

  // Main values
  snprintf(line, sizeof(line),
           "\r\n" ANSI_BOLD "Tension:" ANSI_RESET
           " %6.2f kg  (SP: %.2f)     \r\n",
           data->tension_actual, data->tension_setpoint);
  lcd_tx(handle, line);

  snprintf(line, sizeof(line),
           ANSI_BOLD "Speed:  " ANSI_RESET " %6.0f RPM               \r\n",
           data->speed_actual);
  lcd_tx(handle, line);

  snprintf(line, sizeof(line),
           ANSI_BOLD "PWM:    " ANSI_RESET " %5.1f %%                 \r\n",
           data->pwm_percent);
  lcd_tx(handle, line);

  // Calibration status
  snprintf(line, sizeof(line), "\r\nCalibrated: %s     \r\n",
           data->calibrated ? ANSI_GREEN "YES" ANSI_RESET
                            : ANSI_RED "NO" ANSI_RESET);
  lcd_tx(handle, line);

  // Faults
  if (data->fault_flags != 0) {
    snprintf(line, sizeof(line),
             ANSI_RED "FAULTS: 0x%04X" ANSI_RESET "     \r\n",
             data->fault_flags);
    lcd_tx(handle, line);
  } else {
    lcd_tx(handle, "                           \r\n");
  }

  // Message area
  if (handle->message[0] != '\0') {
    snprintf(line, sizeof(line), "\r\n" ANSI_YELLOW "%s" ANSI_RESET "     \r\n",
             handle->message);
    lcd_tx(handle, line);
  }

  // Menu
  lcd_tx(handle, "\r\n" ANSI_BOLD "Commands:" ANSI_RESET "\r\n");
  lcd_tx(handle, "  R - Run/Start\r\n");
  lcd_tx(handle, "  S - Stop\r\n");
  lcd_tx(handle, "  T - Tune (auto)\r\n");
  lcd_tx(handle, "  C - Calibrate\r\n");
  lcd_tx(handle, "  +/- - Adj setpoint\r\n");
  lcd_tx(handle, "\r\n> ");
}

void lcd_set_screen(lcd_handle_t handle, menu_screen_t screen) {
  if (handle) {
    handle->current_screen = screen;
    lcd_clear(handle);
  }
}

menu_screen_t lcd_get_screen(lcd_handle_t handle) {
  return (handle != NULL) ? handle->current_screen : MENU_MAIN;
}

void lcd_set_callback(lcd_handle_t handle, lcd_command_callback_t callback,
                      void *user_data) {
  if (handle) {
    handle->callback = callback;
    handle->callback_user_data = user_data;
  }
}

void lcd_process_input(lcd_handle_t handle) {
  if (handle == NULL)
    return;

  uint8_t buf[32];
  int len = uart_read_bytes(handle->uart_num, buf, sizeof(buf), 0);

  for (int i = 0; i < len; i++) {
    char c = buf[i];

    // Echo
    uart_write_bytes(handle->uart_num, &c, 1);

    if (c == '\r' || c == '\n') {
      // Process command
      if (handle->input_pos > 0) {
        handle->input_buffer[handle->input_pos] = '\0';

        // Parse and execute
        char cmd = handle->input_buffer[0];
        float value = 0;
        if (handle->input_pos > 1) {
          sscanf(&handle->input_buffer[1], "%f", &value);
        }

        if (handle->callback) {
          handle->callback(cmd, value, handle->callback_user_data);
        }
      }
      handle->input_pos = 0;
    } else if (c == 0x7F || c == 0x08) {
      // Backspace
      if (handle->input_pos > 0) {
        handle->input_pos--;
        lcd_tx(handle, "\b \b");
      }
    } else if (handle->input_pos < sizeof(handle->input_buffer) - 1) {
      handle->input_buffer[handle->input_pos++] = c;
    }
  }
}

void lcd_show_message(lcd_handle_t handle, const char *msg,
                      uint16_t duration_ms) {
  if (handle && msg) {
    strncpy(handle->message, msg, sizeof(handle->message) - 1);
    handle->message[sizeof(handle->message) - 1] = '\0';
    handle->message_expire = xTaskGetTickCount() + pdMS_TO_TICKS(duration_ms);
  }
}

void lcd_clear(lcd_handle_t handle) {
  if (handle) {
    lcd_tx(handle, ANSI_CLEAR ANSI_HOME);
  }
}
