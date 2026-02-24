/**
 * @file buttons.c
 * @brief Button Handler Implementation
 */

#include "buttons.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <string.h>


static const char *TAG = "buttons";

typedef struct {
  gpio_num_t gpio;
  bool pressed;
  bool prev_raw;
  int64_t press_time_us;
  uint8_t debounce_count;
  bool long_press_triggered;
} button_state_t;

struct buttons_s {
  button_state_t run;
  button_state_t stop;
  button_state_t estop;

  bool estop_latched;
  uint32_t debounce_ms;
  uint32_t long_press_ms;

  QueueHandle_t event_queue;
  button_callback_t callback;
  void *callback_user_data;
};

static void IRAM_ATTR button_isr_handler(void *arg);

button_config_t buttons_get_default_config(void) {
  button_config_t config = {
      .gpio_run = 10,
      .gpio_stop = 11,
      .gpio_estop = 12,
      .debounce_ms = 50,
      .long_press_ms = 3000,
  };
  return config;
}

esp_err_t buttons_init(buttons_handle_t *handle,
                       const button_config_t *config) {
  if (handle == NULL || config == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  struct buttons_s *btns = calloc(1, sizeof(struct buttons_s));
  if (btns == NULL) {
    return ESP_ERR_NO_MEM;
  }

  btns->run.gpio = config->gpio_run;
  btns->stop.gpio = config->gpio_stop;
  btns->estop.gpio = config->gpio_estop;
  btns->debounce_ms = config->debounce_ms;
  btns->long_press_ms = config->long_press_ms;

  btns->event_queue = xQueueCreate(10, sizeof(button_event_t));
  if (btns->event_queue == NULL) {
    free(btns);
    return ESP_ERR_NO_MEM;
  }

  // Configure GPIOs
  gpio_config_t io_conf = {
      .intr_type = GPIO_INTR_ANYEDGE,
      .mode = GPIO_MODE_INPUT,
      .pin_bit_mask = (1ULL << config->gpio_run) | (1ULL << config->gpio_stop) |
                      (1ULL << config->gpio_estop),
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
  };
  gpio_config(&io_conf);

  // Install ISR service
  gpio_install_isr_service(0);
  gpio_isr_handler_add(config->gpio_estop, button_isr_handler, btns);

  ESP_LOGI(TAG, "Buttons initialized: RUN=%d, STOP=%d, ESTOP=%d",
           config->gpio_run, config->gpio_stop, config->gpio_estop);

  *handle = btns;
  return ESP_OK;
}

void buttons_deinit(buttons_handle_t handle) {
  if (handle) {
    gpio_isr_handler_remove(handle->estop.gpio);
    vQueueDelete(handle->event_queue);
    free(handle);
  }
}

static void send_event(buttons_handle_t handle, button_event_t event) {
  xQueueSend(handle->event_queue, &event, 0);

  if (handle->callback) {
    handle->callback(event, handle->callback_user_data);
  }
}

static void IRAM_ATTR button_isr_handler(void *arg) {
  struct buttons_s *btns = (struct buttons_s *)arg;

  // E-STOP ISR - highest priority
  if (gpio_get_level(btns->estop.gpio) ==
      0) { // NC button - active when open/pressed
    btns->estop_latched = true;
    button_event_t event = BTN_EVENT_ESTOP_ACTIVATED;
    xQueueSendFromISR(btns->event_queue, &event, NULL);
  }
}

void buttons_update(buttons_handle_t handle) {
  if (handle == NULL)
    return;

  int64_t now = esp_timer_get_time();

  // === RUN Button ===
  bool raw_run = (gpio_get_level(handle->run.gpio) == 0);
  if (raw_run != handle->run.prev_raw) {
    handle->run.debounce_count = 0;
  } else {
    handle->run.debounce_count++;
  }
  handle->run.prev_raw = raw_run;

  if (handle->run.debounce_count >= 3) { // 3 * 20ms = 60ms
    if (raw_run != handle->run.pressed) {
      handle->run.pressed = raw_run;
      if (raw_run) {
        handle->run.press_time_us = now;
        send_event(handle, BTN_EVENT_RUN_PRESSED);
      } else {
        send_event(handle, BTN_EVENT_RUN_RELEASED);
      }
    }
  }

  // === STOP Button ===
  bool raw_stop = (gpio_get_level(handle->stop.gpio) == 0);
  if (raw_stop != handle->stop.prev_raw) {
    handle->stop.debounce_count = 0;
  } else {
    handle->stop.debounce_count++;
  }
  handle->stop.prev_raw = raw_stop;

  if (handle->stop.debounce_count >= 3) {
    if (raw_stop != handle->stop.pressed) {
      handle->stop.pressed = raw_stop;
      handle->stop.long_press_triggered = false;
      if (raw_stop) {
        handle->stop.press_time_us = now;
        send_event(handle, BTN_EVENT_STOP_PRESSED);
      } else {
        send_event(handle, BTN_EVENT_STOP_RELEASED);
      }
    }

    // Long press detection
    if (handle->stop.pressed && !handle->stop.long_press_triggered) {
      if ((now - handle->stop.press_time_us) / 1000 > handle->long_press_ms) {
        handle->stop.long_press_triggered = true;
        send_event(handle, BTN_EVENT_STOP_LONG_PRESS);
      }
    }
  }

  // === E-STOP Check ===
  bool estop_hw = (gpio_get_level(handle->estop.gpio) == 0);
  if (estop_hw && !handle->estop_latched) {
    handle->estop_latched = true;
    send_event(handle, BTN_EVENT_ESTOP_ACTIVATED);
  }

  if (!estop_hw && handle->estop_latched && handle->estop.pressed) {
    // Hardware released - can be software reset
    send_event(handle, BTN_EVENT_ESTOP_RESET);
    handle->estop.pressed = false;
  }
  handle->estop.pressed = estop_hw;
}

void buttons_register_callback(buttons_handle_t handle,
                               button_callback_t callback, void *user_data) {
  if (handle) {
    handle->callback = callback;
    handle->callback_user_data = user_data;
  }
}

button_event_t buttons_get_event(buttons_handle_t handle, uint32_t timeout_ms) {
  if (handle == NULL)
    return BTN_EVENT_NONE;

  button_event_t event;
  if (xQueueReceive(handle->event_queue, &event, pdMS_TO_TICKS(timeout_ms))) {
    return event;
  }
  return BTN_EVENT_NONE;
}

bool buttons_is_estop_active(buttons_handle_t handle) {
  return handle != NULL && handle->estop_latched;
}

esp_err_t buttons_reset_estop(buttons_handle_t handle) {
  if (handle == NULL)
    return ESP_ERR_INVALID_ARG;

  // Can only reset if hardware is released
  if (gpio_get_level(handle->estop.gpio) == 1) {
    handle->estop_latched = false;
    ESP_LOGI(TAG, "E-STOP reset");
    return ESP_OK;
  }

  ESP_LOGW(TAG, "Cannot reset E-STOP - hardware still active");
  return ESP_ERR_INVALID_STATE;
}

bool buttons_get_run_state(buttons_handle_t handle) {
  return handle != NULL && handle->run.pressed;
}

bool buttons_get_stop_state(buttons_handle_t handle) {
  return handle != NULL && handle->stop.pressed;
}
