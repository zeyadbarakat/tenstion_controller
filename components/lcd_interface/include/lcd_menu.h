/**
 * @file lcd_menu.h
 * @brief 20x4 I2C LCD + 1x4 Keypad Interface
 *
 * Drives a HD44780 LCD via PCF8574 I2C backpack and reads
 * a 1x4 membrane keypad with debounce + long-press detection.
 */

#ifndef LCD_MENU_H
#define LCD_MENU_H

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
 * @brief Menu screens
 */
typedef enum {
  SCREEN_MAIN = 0,    /**< Main status display */
  SCREEN_MENU,        /**< Menu list */
  SCREEN_CALIBRATION, /**< Calibration submenu */
  SCREEN_AUTOTUNE,    /**< Auto-tune submenu */
  SCREEN_CONFIG,      /**< Configuration submenu */
  SCREEN_PI_GAINS,    /**< PI gains view/edit */
  SCREEN_FAULTS,      /**< Fault display/clear */
  SCREEN_SYSINFO,     /**< System info */
  SCREEN_EDIT_VALUE,  /**< Value editing overlay */
} menu_screen_t;

/* Keep old enum name for compatibility */
#define MENU_MAIN SCREEN_MAIN
#define MENU_SETTINGS SCREEN_CONFIG
#define MENU_CALIBRATION SCREEN_CALIBRATION
#define MENU_TUNING SCREEN_AUTOTUNE
#define MENU_DIAGNOSTICS SCREEN_SYSINFO
#define MENU_RUN SCREEN_MAIN

/**
 * @brief Keypad events
 */
typedef enum {
  KEY_NONE = 0,
  KEY_1_SHORT,   /**< ▲ short press */
  KEY_2_SHORT,   /**< ▼ short press */
  KEY_3_SHORT,   /**< ◄ short press */
  KEY_4_SHORT,   /**< ► short press */
  KEY_1_LONG,    /**< ▲ long press (>500ms) */
  KEY_2_LONG,    /**< ▼ long press */
  KEY_3_HOLD,    /**< ◄ held down */
  KEY_4_HOLD,    /**< ► held down */
  KEY_3_RELEASE, /**< ◄ released (stop jog) */
  KEY_4_RELEASE, /**< ► released (stop jog) */
} keypad_event_t;

/**
 * @brief LCD command from menu actions
 */
typedef enum {
  LCD_CMD_NONE = 0,
  LCD_CMD_TENSION_UP,
  LCD_CMD_TENSION_DOWN,
  LCD_CMD_JOG_LEFT,
  LCD_CMD_JOG_RIGHT,
  LCD_CMD_JOG_STOP,
  LCD_CMD_RUN,
  LCD_CMD_STOP,
  LCD_CMD_TARE,
  LCD_CMD_CALIBRATE,
  LCD_CMD_AUTOTUNE_SPEED,
  LCD_CMD_AUTOTUNE_TENSION,
  LCD_CMD_CLEAR_FAULTS,
  LCD_CMD_SET_PPR,
  LCD_CMD_SET_MAX_RPM,
  LCD_CMD_SET_MOTOR_DIR,
  LCD_CMD_SET_TENSION_STEP,
  LCD_CMD_SET_SPEED_KP,
  LCD_CMD_SET_SPEED_KI,
  LCD_CMD_SET_TENSION_KP,
  LCD_CMD_SET_TENSION_KI,
} lcd_command_id_t;

/**
 * @brief Display data pushed to LCD from control manager
 */
typedef struct {
  float tension_setpoint;
  float tension_actual;
  float speed_actual;
  float speed_setpoint;
  float pwm_percent;
  uint8_t system_state;
  uint16_t fault_flags;
  bool calibrated;
  bool tuned;
  uint32_t uptime_seconds;
  /* PI gains for display */
  float speed_kp;
  float speed_ki;
  float tension_kp;
  float tension_ki;
  /* Config values for display */
  uint16_t encoder_ppr;
  uint16_t max_rpm;
  float tension_step;
} lcd_display_data_t;

/**
 * @brief LCD configuration
 */
typedef struct {
  int sda_gpio;
  int scl_gpio;
  uint8_t i2c_addr;
  int key_gpios[4]; /**< KEY1, KEY2, KEY3, KEY4 */
} lcd_config_t;

/**
 * @brief LCD handle
 */
typedef struct lcd_menu_s *lcd_handle_t;

/**
 * @brief Command callback (old signature kept for compat)
 */
typedef void (*lcd_command_callback_t)(uint8_t command, float value,
                                       void *user_data);

/**
 * @brief New command callback with structured command
 */
typedef void (*lcd_menu_command_callback_t)(lcd_command_id_t cmd, float value,
                                            void *user_data);

/*******************************************************************************
 * Public API Functions
 ******************************************************************************/

esp_err_t lcd_init(lcd_handle_t *handle, const lcd_config_t *config);
void lcd_deinit(lcd_handle_t handle);
void lcd_update(lcd_handle_t handle, const lcd_display_data_t *data);
void lcd_process_input(lcd_handle_t handle);
void lcd_set_screen(lcd_handle_t handle, menu_screen_t screen);
menu_screen_t lcd_get_screen(lcd_handle_t handle);
void lcd_set_callback(lcd_handle_t handle, lcd_command_callback_t callback,
                      void *user_data);
void lcd_set_menu_callback(lcd_handle_t handle,
                           lcd_menu_command_callback_t callback,
                           void *user_data);
void lcd_show_message(lcd_handle_t handle, const char *msg,
                      uint16_t duration_ms);
void lcd_clear(lcd_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* LCD_MENU_H */
