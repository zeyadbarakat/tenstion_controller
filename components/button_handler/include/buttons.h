/**
 * @file buttons.h
 * @brief Button Handler with Debouncing and E-STOP Support
 */

#ifndef BUTTONS_H
#define BUTTONS_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  BTN_EVENT_NONE = 0,
  BTN_EVENT_RUN_PRESSED,
  BTN_EVENT_RUN_RELEASED,
  BTN_EVENT_STOP_PRESSED,
  BTN_EVENT_STOP_RELEASED,
  BTN_EVENT_STOP_LONG_PRESS,
  BTN_EVENT_ESTOP_ACTIVATED,
  BTN_EVENT_ESTOP_RESET
} button_event_t;

typedef void (*button_callback_t)(button_event_t event, void *user_data);

typedef struct {
  int gpio_run;
  int gpio_stop;
  int gpio_estop;
  uint32_t debounce_ms;
  uint32_t long_press_ms;
} button_config_t;

typedef struct buttons_s *buttons_handle_t;

esp_err_t buttons_init(buttons_handle_t *handle, const button_config_t *config);
void buttons_deinit(buttons_handle_t handle);
void buttons_register_callback(buttons_handle_t handle,
                               button_callback_t callback, void *user_data);
button_event_t buttons_get_event(buttons_handle_t handle, uint32_t timeout_ms);
bool buttons_is_estop_active(buttons_handle_t handle);
esp_err_t buttons_reset_estop(buttons_handle_t handle);
bool buttons_get_run_state(buttons_handle_t handle);
bool buttons_get_stop_state(buttons_handle_t handle);
void buttons_update(buttons_handle_t handle);
button_config_t buttons_get_default_config(void);

#ifdef __cplusplus
}
#endif

#endif /* BUTTONS_H */
