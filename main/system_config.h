/**
 * @file system_config.h
 * @brief System-wide configuration and pin definitions for Tension Controller
 * 
 * This header provides centralized configuration using Kconfig values
 * with fallback defaults. All hardware-related constants are defined here.
 */

#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include "sdkconfig.h"

/*******************************************************************************
 * GPIO PIN ASSIGNMENTS
 ******************************************************************************/

// Encoder Pins (PCNT capable)
#define PIN_ENCODER_A           CONFIG_ENCODER_GPIO_A
#define PIN_ENCODER_B           CONFIG_ENCODER_GPIO_B

// HX711 Load Cell Pins
#define PIN_HX711_DATA          CONFIG_HX711_GPIO_DATA
#define PIN_HX711_CLOCK         CONFIG_HX711_GPIO_CLOCK

// Motor Control Pins
#define PIN_MOTOR_PWM           CONFIG_MOTOR_GPIO_PWM
#define PIN_MOTOR_DIR           CONFIG_MOTOR_GPIO_DIR

// I2C LCD Pins
#define PIN_LCD_SDA             CONFIG_LCD_GPIO_SDA
#define PIN_LCD_SCL             CONFIG_LCD_GPIO_SCL

// Button Pins (Active Low with Pull-up)
#define PIN_BTN_RUN             CONFIG_BTN_GPIO_RUN
#define PIN_BTN_STOP            CONFIG_BTN_GPIO_STOP
#define PIN_BTN_ESTOP           CONFIG_BTN_GPIO_ESTOP

// LED Pins
#define PIN_LED_RUNNING         CONFIG_LED_GPIO_RUNNING
#define PIN_LED_FAULT           CONFIG_LED_GPIO_FAULT

/*******************************************************************************
 * ENCODER CONFIGURATION
 ******************************************************************************/

#define ENCODER_PPR                 CONFIG_ENCODER_PPR          // Pulses per revolution
#define ENCODER_CPR                 (ENCODER_PPR * 4)           // Counts per revolution (4x quadrature)
#define ENCODER_FILTER_SIZE         CONFIG_ENCODER_FILTER_SIZE  // Moving average window

/*******************************************************************************
 * MOTOR CONFIGURATION
 ******************************************************************************/

#define MOTOR_PWM_FREQ_HZ           CONFIG_MOTOR_PWM_FREQ_HZ    // PWM frequency
#define MOTOR_PWM_RESOLUTION        10                           // 10-bit (0-1023)
#define MOTOR_PWM_MAX_DUTY          ((1 << MOTOR_PWM_RESOLUTION) - 1)  // 1023
#define MOTOR_MAX_RPM               CONFIG_MOTOR_MAX_RPM

/*******************************************************************************
 * LOAD CELL (HX711) CONFIGURATION
 ******************************************************************************/

#define HX711_FILTER_SIZE           CONFIG_HX711_FILTER_SIZE
#define HX711_GAIN                  128                          // Channel A, Gain 128
#define HX711_CLOCK_PULSES          (24 + 1)                     // 24 data + 1 for gain 128

/*******************************************************************************
 * CONTROL LOOP CONFIGURATION
 ******************************************************************************/

// Loop frequencies
#define TENSION_LOOP_FREQ_HZ        CONFIG_TENSION_LOOP_FREQ_HZ
#define SPEED_LOOP_FREQ_HZ          CONFIG_SPEED_LOOP_FREQ_HZ

// Sample times (in seconds)
#define TENSION_LOOP_DT             (1.0f / TENSION_LOOP_FREQ_HZ)
#define SPEED_LOOP_DT               (1.0f / SPEED_LOOP_FREQ_HZ)

// Default PI gains (from Kconfig, scaled by 1000)
#define SPEED_KP_DEFAULT            (CONFIG_SPEED_KP_DEFAULT / 1000.0f)
#define SPEED_KI_DEFAULT            (CONFIG_SPEED_KI_DEFAULT / 1000.0f)
#define TENSION_KP_DEFAULT          (CONFIG_TENSION_KP_DEFAULT / 1000.0f)
#define TENSION_KI_DEFAULT          (CONFIG_TENSION_KI_DEFAULT / 1000.0f)

// Speed loop output limits (RPM)
#define SPEED_OUTPUT_MIN            0.0f
#define SPEED_OUTPUT_MAX            100.0f      // PWM percentage

// Tension loop output limits (Speed setpoint in RPM)
#define TENSION_OUTPUT_MIN          0.0f
#define TENSION_OUTPUT_MAX          ((float)MOTOR_MAX_RPM)

/*******************************************************************************
 * SAFETY LIMITS
 ******************************************************************************/

#define SAFETY_MAX_TENSION_KG       (CONFIG_SAFETY_MAX_TENSION_KG / 10.0f)
#define SAFETY_MIN_TENSION_KG       (CONFIG_SAFETY_MIN_TENSION_KG / 10.0f)
#define SAFETY_MAX_RPM              CONFIG_SAFETY_MAX_RPM
#define SAFETY_STALL_TIMEOUT_MS     CONFIG_SAFETY_STALL_TIMEOUT_MS
#define SAFETY_ENCODER_TIMEOUT_MS   2000        // Encoder failure detection
#define SAFETY_WARNING_THRESHOLD    0.9f        // 90% of limit triggers warning

/*******************************************************************************
 * LCD CONFIGURATION
 ******************************************************************************/

#define LCD_I2C_ADDRESS             CONFIG_LCD_I2C_ADDRESS
#define LCD_ROWS                    CONFIG_LCD_ROWS
#define LCD_COLS                    CONFIG_LCD_COLS
#define LCD_BACKLIGHT_TIMEOUT_S     CONFIG_LCD_BACKLIGHT_TIMEOUT_S
#define LCD_I2C_FREQ_HZ             100000      // 100kHz I2C

/*******************************************************************************
 * DATA LOGGER CONFIGURATION
 ******************************************************************************/

#define LOGGER_BUFFER_SIZE          CONFIG_LOGGER_BUFFER_SIZE
#define LOGGER_RATE_HZ              CONFIG_LOGGER_RATE_HZ

/*******************************************************************************
 * WIFI CONFIGURATION
 ******************************************************************************/

#define WIFI_SSID                   CONFIG_WIFI_SSID
#define WIFI_PASSWORD               CONFIG_WIFI_PASSWORD
#define WIFI_AP_SSID                CONFIG_WIFI_AP_SSID
#define WIFI_AP_PASSWORD            CONFIG_WIFI_AP_PASSWORD
#define WIFI_AP_MODE                CONFIG_WIFI_AP_MODE

/*******************************************************************************
 * FEATURE ENABLES
 ******************************************************************************/

#define FEATURE_WEB_INTERFACE       CONFIG_ENABLE_WEB_INTERFACE
#define FEATURE_LCD_INTERFACE       CONFIG_ENABLE_LCD_INTERFACE
#define FEATURE_AUTO_TUNING         CONFIG_ENABLE_AUTO_TUNING
#define FEATURE_DATA_LOGGER         CONFIG_ENABLE_DATA_LOGGER

/*******************************************************************************
 * FREERTOS TASK CONFIGURATION
 ******************************************************************************/

// Task stack sizes (in bytes)
#define TASK_STACK_MAIN             8192
#define TASK_STACK_SAFETY           4096
#define TASK_STACK_ENCODER          2048
#define TASK_STACK_SPEED_CTRL       4096
#define TASK_STACK_LOADCELL         2048
#define TASK_STACK_TENSION_CTRL     4096
#define TASK_STACK_LCD_UI           4096
#define TASK_STACK_WEB              8192
#define TASK_STACK_LOGGER           4096
#define TASK_STACK_BUTTONS          2048

// Task priorities (higher = more important)
#define TASK_PRIO_SAFETY            15      // Highest priority
#define TASK_PRIO_ENCODER           11
#define TASK_PRIO_SPEED_CTRL        10
#define TASK_PRIO_LOADCELL          9
#define TASK_PRIO_TENSION_CTRL      8
#define TASK_PRIO_BUTTONS           6
#define TASK_PRIO_LCD_UI            5
#define TASK_PRIO_WEB               4
#define TASK_PRIO_LOGGER            3

// Core affinity
#define CORE_PROTOCOL               0       // WiFi, LCD, logging
#define CORE_APPLICATION            1       // Control loops, safety

/*******************************************************************************
 * TIMING CONSTANTS
 ******************************************************************************/

#define MS_TO_TICKS(ms)             ((ms) / portTICK_PERIOD_MS)
#define US_TO_MS(us)                ((us) / 1000)

// Debounce timing
#define BUTTON_DEBOUNCE_MS          50
#define BUTTON_LONG_PRESS_MS        3000

// Auto-tune timing
#define AUTOTUNE_TIMEOUT_S          120     // 2 minute timeout
#define AUTOTUNE_MIN_CYCLES         3       // Minimum oscillation cycles

/*******************************************************************************
 * NVS NAMESPACE DEFINITIONS
 ******************************************************************************/

#define NVS_NAMESPACE_LOADCELL      "loadcell"
#define NVS_NAMESPACE_CONTROL       "control"
#define NVS_NAMESPACE_SAFETY        "safety"
#define NVS_NAMESPACE_FAULT_LOG     "faults"

/*******************************************************************************
 * MATHEMATICAL CONSTANTS
 ******************************************************************************/

#ifndef M_PI
#define M_PI                        3.14159265358979323846f
#endif

#define CLAMP(x, min, max)          ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define ABS(x)                      ((x) < 0 ? -(x) : (x))

#endif /* SYSTEM_CONFIG_H */
