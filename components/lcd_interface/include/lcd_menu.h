/**
 * @file lcd_menu.h
 * @brief UART-based LCD Terminal Interface
 *
 * Uses ANSI escape codes for cursor control and formatting.
 * Compatible with most serial terminals (PuTTY, screen, minicom).
 */

#ifndef LCD_MENU_H
#define LCD_MENU_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  MENU_MAIN = 0,
  MENU_SETTINGS,
  MENU_CALIBRATION,
  MENU_TUNING,
  MENU_DIAGNOSTICS,
  MENU_RUN
} menu_screen_t;

typedef struct {
  float tension_setpoint;
  float tension_actual;
  float speed_actual;
  float pwm_percent;
  uint8_t system_state;
  uint16_t fault_flags;
  bool calibrated;
} lcd_display_data_t;

typedef struct lcd_menu_s *lcd_handle_t;

typedef void (*lcd_command_callback_t)(uint8_t command, float value,
                                       void *user_data);

esp_err_t lcd_init(lcd_handle_t *handle, int uart_num);
void lcd_deinit(lcd_handle_t handle);
void lcd_update(lcd_handle_t handle, const lcd_display_data_t *data);
void lcd_set_screen(lcd_handle_t handle, menu_screen_t screen);
menu_screen_t lcd_get_screen(lcd_handle_t handle);
void lcd_set_callback(lcd_handle_t handle, lcd_command_callback_t callback,
                      void *user_data);
void lcd_process_input(lcd_handle_t handle);
void lcd_show_message(lcd_handle_t handle, const char *msg,
                      uint16_t duration_ms);
void lcd_clear(lcd_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* LCD_MENU_H */
