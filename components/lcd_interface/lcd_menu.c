/**
 * @file lcd_menu.c
 * @brief 20x4 I2C LCD (PCF8574) + 1x4 Membrane Keypad Interface
 *
 * I2C LCD driver (4-bit mode via PCF8574), keypad with debounce
 * and long-press detection, status screens, and menu system.
 */

#include "lcd_menu.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "lcd";

/*******************************************************************************
 * PCF8574 LCD Constants
 ******************************************************************************/

/* PCF8574 bit mapping for most common LCD backpacks:
 *   P0 = RS
 *   P1 = RW
 *   P2 = EN
 *   P3 = Backlight
 *   P4-P7 = D4-D7
 */
#define LCD_RS (1 << 0)
#define LCD_RW (1 << 1)
#define LCD_EN (1 << 2)
#define LCD_BL (1 << 3)

/* HD44780 commands */
#define LCD_CMD_CLEAR 0x01
#define LCD_CMD_HOME 0x02
#define LCD_CMD_ENTRY_MODE 0x06
#define LCD_CMD_DISPLAY_ON 0x0C
#define LCD_CMD_DISPLAY_OFF 0x08
#define LCD_CMD_FUNCTION_SET 0x28 /* 4-bit, 2-line, 5x8 */
#define LCD_CMD_SET_CGRAM 0x40
#define LCD_CMD_SET_DDRAM 0x80

/* Row offsets for 20x4 LCD */
static const uint8_t ROW_OFFSETS[4] = {0x00, 0x40, 0x14, 0x54};

#define LCD_COLS 20
#define LCD_ROWS 4

/*******************************************************************************
 * Keypad Constants
 ******************************************************************************/

#define NUM_KEYS 4
#define DEBOUNCE_MS 50
#define LONG_PRESS_MS 500
#define REPEAT_MS 200

/*******************************************************************************
 * Menu Constants
 ******************************************************************************/

#define MENU_ITEMS_COUNT 6

static const char *MENU_ITEMS[MENU_ITEMS_COUNT] = {
    "Calibration", "Auto-Tune", "Configuration",
    "PI Gains",    "Faults",    "System Info",
};

static const menu_screen_t MENU_SCREENS[MENU_ITEMS_COUNT] = {
    SCREEN_CALIBRATION, SCREEN_AUTOTUNE, SCREEN_CONFIG,
    SCREEN_PI_GAINS,    SCREEN_FAULTS,   SCREEN_SYSINFO,
};

/* Sub-menu item counts */
#define CAL_ITEMS 2
#define TUNE_ITEMS 2
#define CONFIG_ITEMS 4
#define PI_ITEMS 4
#define FAULT_ITEMS 3
#define SYSINFO_ITEMS 4

/*******************************************************************************
 * Internal State
 ******************************************************************************/

typedef struct {
  bool pressed;
  bool was_pressed;
  uint32_t press_start_ms;
  bool long_fired;
  bool hold_active;
} key_state_t;

struct lcd_menu_s {
  /* I2C */
  i2c_master_bus_handle_t i2c_bus;
  i2c_master_dev_handle_t i2c_dev;
  uint8_t backlight;

  /* Keypad */
  int key_gpios[NUM_KEYS];
  key_state_t keys[NUM_KEYS];

  /* Screen state */
  menu_screen_t current_screen;
  int menu_cursor; /* Cursor position in current menu */
  int menu_scroll; /* Scroll offset */
  lcd_display_data_t data;

  /* Message overlay */
  char message[21];
  int64_t message_expire_us;

  /* Value editor */
  float edit_value;
  float edit_min;
  float edit_max;
  float edit_step;
  const char *edit_label;
  lcd_command_id_t edit_cmd;

  /* Callbacks */
  lcd_command_callback_t old_callback;
  void *old_cb_data;
  lcd_menu_command_callback_t menu_callback;
  void *menu_cb_data;
};

/*******************************************************************************
 * I2C LCD Low-Level Driver
 ******************************************************************************/

static esp_err_t lcd_i2c_write_byte(lcd_handle_t h, uint8_t val) {
  return i2c_master_transmit(h->i2c_dev, &val, 1, 50);
}

static void lcd_pulse_enable(lcd_handle_t h, uint8_t val) {
  lcd_i2c_write_byte(h, val | LCD_EN | h->backlight);
  esp_rom_delay_us(1);
  lcd_i2c_write_byte(h, (val & ~LCD_EN) | h->backlight);
  esp_rom_delay_us(50);
}

static void lcd_send_nibble(lcd_handle_t h, uint8_t nibble, uint8_t rs) {
  uint8_t val = (nibble & 0xF0) | rs | h->backlight;
  lcd_pulse_enable(h, val);
}

static void lcd_send_byte(lcd_handle_t h, uint8_t byte, uint8_t rs) {
  lcd_send_nibble(h, byte & 0xF0, rs);
  lcd_send_nibble(h, (byte << 4) & 0xF0, rs);
}

static void lcd_command(lcd_handle_t h, uint8_t cmd) {
  lcd_send_byte(h, cmd, 0);
  if (cmd == LCD_CMD_CLEAR || cmd == LCD_CMD_HOME) {
    esp_rom_delay_us(2000);
  }
}

static void lcd_write_char(lcd_handle_t h, char c) {
  lcd_send_byte(h, (uint8_t)c, LCD_RS);
}

static void lcd_set_cursor(lcd_handle_t h, uint8_t col, uint8_t row) {
  if (row >= LCD_ROWS)
    row = 0;
  if (col >= LCD_COLS)
    col = 0;
  lcd_command(h, LCD_CMD_SET_DDRAM | (ROW_OFFSETS[row] + col));
}

static void lcd_print(lcd_handle_t h, const char *str) {
  while (*str) {
    lcd_write_char(h, *str++);
  }
}

static void lcd_print_line(lcd_handle_t h, uint8_t row, const char *text) {
  char buf[LCD_COLS + 1];
  int len = strlen(text);
  if (len > LCD_COLS)
    len = LCD_COLS;
  memcpy(buf, text, len);
  /* Pad with spaces to clear rest of line */
  for (int i = len; i < LCD_COLS; i++) {
    buf[i] = ' ';
  }
  buf[LCD_COLS] = '\0';
  lcd_set_cursor(h, 0, row);
  lcd_print(h, buf);
}

/* Create custom arrow character for menu cursor */
static void lcd_create_char(lcd_handle_t h, uint8_t location,
                            const uint8_t *charmap) {
  lcd_command(h, LCD_CMD_SET_CGRAM | (location << 3));
  for (int i = 0; i < 8; i++) {
    lcd_send_byte(h, charmap[i], LCD_RS);
  }
  lcd_command(h, LCD_CMD_SET_DDRAM); /* Back to DDRAM */
}

static esp_err_t lcd_hw_init(lcd_handle_t h, const lcd_config_t *config) {
  /* Configure I2C master bus */
  i2c_master_bus_config_t bus_cfg = {
      .i2c_port = I2C_NUM_0,
      .sda_io_num = config->sda_gpio,
      .scl_io_num = config->scl_gpio,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };

  esp_err_t ret = i2c_new_master_bus(&bus_cfg, &h->i2c_bus);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(ret));
    return ret;
  }

  /* Add LCD device */
  i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = config->i2c_addr,
      .scl_speed_hz = 100000,
  };

  ret = i2c_master_bus_add_device(h->i2c_bus, &dev_cfg, &h->i2c_dev);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "I2C device add failed: %s", esp_err_to_name(ret));
    return ret;
  }

  h->backlight = LCD_BL;

  /* HD44780 init sequence (4-bit mode) - per datasheet timing */
  vTaskDelay(pdMS_TO_TICKS(50)); /* Wait >40ms after power-on */

  /* Send 0x30 three times to ensure 8-bit mode start */
  lcd_send_nibble(h, 0x30, 0);
  vTaskDelay(pdMS_TO_TICKS(5));
  lcd_send_nibble(h, 0x30, 0);
  vTaskDelay(pdMS_TO_TICKS(1));
  lcd_send_nibble(h, 0x30, 0);
  esp_rom_delay_us(150);

  /* Switch to 4-bit mode */
  lcd_send_nibble(h, 0x20, 0);
  esp_rom_delay_us(150);

  /* Now in 4-bit mode - configure display */
  lcd_command(h, LCD_CMD_FUNCTION_SET); /* 4-bit, 2 lines, 5x8 */
  lcd_command(h, LCD_CMD_DISPLAY_OFF);
  lcd_command(h, LCD_CMD_CLEAR);
  lcd_command(h, LCD_CMD_ENTRY_MODE);
  lcd_command(h, LCD_CMD_DISPLAY_ON);

  /* Create custom char 0: right arrow for menu cursor */
  static const uint8_t arrow[] = {0x00, 0x04, 0x06, 0x1F,
                                  0x06, 0x04, 0x00, 0x00};
  lcd_create_char(h, 0, arrow);

  ESP_LOGI(TAG, "20x4 LCD initialized (I2C addr=0x%02X)", config->i2c_addr);
  return ESP_OK;
}

/*******************************************************************************
 * Keypad Driver
 ******************************************************************************/

static void keypad_init(lcd_handle_t h, const int gpios[4]) {
  for (int i = 0; i < NUM_KEYS; i++) {
    h->key_gpios[i] = gpios[i];
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << gpios[i]),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    memset(&h->keys[i], 0, sizeof(key_state_t));
  }
  ESP_LOGI(TAG, "Keypad initialized: GPIO %d,%d,%d,%d", gpios[0], gpios[1],
           gpios[2], gpios[3]);
}

static uint32_t millis(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

/**
 * @brief Scan keypad and return event
 */
static keypad_event_t keypad_scan(lcd_handle_t h) {
  uint32_t now = millis();
  keypad_event_t event = KEY_NONE;

  for (int i = 0; i < NUM_KEYS; i++) {
    key_state_t *k = &h->keys[i];
    bool raw = (gpio_get_level(h->key_gpios[i]) == 0); /* Active low */

    if (raw && !k->pressed) {
      /* Key just pressed — start debounce */
      if ((now - k->press_start_ms) > DEBOUNCE_MS) {
        k->pressed = true;
        k->press_start_ms = now;
        k->long_fired = false;
        k->hold_active = false;
      }
    } else if (!raw && k->pressed) {
      /* Key released */
      k->pressed = false;
      uint32_t duration = now - k->press_start_ms;

      if (k->hold_active) {
        /* Key 3 or 4 was held — send release event */
        if (i == 2)
          event = KEY_3_RELEASE;
        else if (i == 3)
          event = KEY_4_RELEASE;
      } else if (!k->long_fired && duration < LONG_PRESS_MS) {
        /* Short press */
        event = (keypad_event_t)(KEY_1_SHORT + i);
      }
      k->hold_active = false;
      k->long_fired = false;
    } else if (raw && k->pressed) {
      /* Key still held */
      uint32_t duration = now - k->press_start_ms;

      if (!k->long_fired && duration >= LONG_PRESS_MS) {
        k->long_fired = true;
        /* Long press for keys 1,2 — fire once */
        if (i == 0)
          event = KEY_1_LONG;
        else if (i == 1)
          event = KEY_2_LONG;
        /* For keys 3,4 — start hold */
        else {
          k->hold_active = true;
          event = (i == 2) ? KEY_3_HOLD : KEY_4_HOLD;
        }
      }

      /* Repeat hold events for keys 3,4 */
      if (k->hold_active && duration > LONG_PRESS_MS) {
        static uint32_t last_repeat[NUM_KEYS] = {0};
        if ((now - last_repeat[i]) > REPEAT_MS) {
          last_repeat[i] = now;
          event = (i == 2) ? KEY_3_HOLD : KEY_4_HOLD;
        }
      }
    }
  }

  return event;
}

/*******************************************************************************
 * Screen Rendering
 ******************************************************************************/

static const char *state_name(uint8_t state) {
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

static void render_main_screen(lcd_handle_t h) {
  char line[21];

  /* Line 0: Tension */
  snprintf(line, 21, "T:%5.1fkg SP:%5.1fkg", h->data.tension_actual,
           h->data.tension_setpoint);
  lcd_print_line(h, 0, line);

  /* Line 1: Speed + PWM */
  snprintf(line, 21, "S:%5.0frpm PWM:%4.0f%%", h->data.speed_actual,
           h->data.pwm_percent);
  lcd_print_line(h, 1, line);

  /* Line 2: State + fault indicator */
  if (h->data.fault_flags) {
    snprintf(line, 21, "%-10s F:0x%04X", state_name(h->data.system_state),
             h->data.fault_flags);
  } else {
    snprintf(line, 21, "%-10s %s", state_name(h->data.system_state),
             h->data.calibrated ? "CAL" : "!CAL");
  }
  lcd_print_line(h, 2, line);

  /* Line 3: Check for message overlay */
  if (h->message[0] != '\0' && esp_timer_get_time() < h->message_expire_us) {
    lcd_print_line(h, 3, h->message);
  } else {
    h->message[0] = '\0';
    lcd_print_line(h, 3, "\x01\x02:Set \x03\x04:Jog \x01M");
  }
}

static void render_menu_list(lcd_handle_t h, const char *title,
                             const char **items, int count) {
  char line[21];

  /* Line 0: Title */
  snprintf(line, 21, "=== %s ===", title);
  lcd_print_line(h, 0, line);

  /* Lines 1-3: Scrollable items with cursor */
  for (int row = 1; row < LCD_ROWS; row++) {
    int idx = h->menu_scroll + (row - 1);
    if (idx < count) {
      char cursor = (idx == h->menu_cursor) ? '\x00' : ' '; /* custom arrow */
      snprintf(line, 21, "%c %-18s", cursor, items[idx]);
      lcd_print_line(h, row, line);
    } else {
      lcd_print_line(h, row, "");
    }
  }
}

static void render_calibration(lcd_handle_t h) {
  static const char *items[] = {"Tare (Zero)", "Span Calibrate"};
  render_menu_list(h, "CALIBRATION", items, CAL_ITEMS);
}

static void render_autotune(lcd_handle_t h) {
  static const char *items[] = {"Speed Loop", "Tension Loop"};
  render_menu_list(h, "AUTO-TUNE", items, TUNE_ITEMS);
}

static void render_config(lcd_handle_t h) {
  char items_buf[CONFIG_ITEMS][21];
  const char *items[CONFIG_ITEMS];

  snprintf(items_buf[0], 21, "PPR: %u", h->data.encoder_ppr);
  snprintf(items_buf[1], 21, "Max RPM: %u", h->data.max_rpm);
  snprintf(items_buf[2], 21, "Motor Dir: FWD");
  snprintf(items_buf[3], 21, "Tens Step: %.1f kg", h->data.tension_step);

  for (int i = 0; i < CONFIG_ITEMS; i++)
    items[i] = items_buf[i];

  render_menu_list(h, "CONFIG", items, CONFIG_ITEMS);
}

static void render_pi_gains(lcd_handle_t h) {
  char items_buf[PI_ITEMS][21];
  const char *items[PI_ITEMS];

  snprintf(items_buf[0], 21, "Spd Kp: %.4f", h->data.speed_kp);
  snprintf(items_buf[1], 21, "Spd Ki: %.4f", h->data.speed_ki);
  snprintf(items_buf[2], 21, "Ten Kp: %.4f", h->data.tension_kp);
  snprintf(items_buf[3], 21, "Ten Ki: %.4f", h->data.tension_ki);

  for (int i = 0; i < PI_ITEMS; i++)
    items[i] = items_buf[i];

  render_menu_list(h, "PI GAINS", items, PI_ITEMS);
}

static const char *fault_name(uint16_t flag) {
  if (flag & 0x0001)
    return "OVER-TENSION";
  if (flag & 0x0002)
    return "OVER-SPEED";
  if (flag & 0x0004)
    return "MOTOR-STALL";
  if (flag & 0x0008)
    return "ENCODER-FAIL";
  if (flag & 0x0010)
    return "LOADCELL-FAIL";
  if (flag & 0x0020)
    return "E-STOP";
  if (flag & 0x0040)
    return "WATCHDOG";
  return "UNKNOWN";
}

static void render_faults(lcd_handle_t h) {
  char line[21];

  lcd_print_line(h, 0, "=== FAULTS ===");

  if (h->data.fault_flags == 0) {
    lcd_print_line(h, 1, "  No active faults");
    lcd_print_line(h, 2, "");
  } else {
    snprintf(line, 21, "  Active: 0x%04X", h->data.fault_flags);
    lcd_print_line(h, 1, line);
    snprintf(line, 21, "  %s", fault_name(h->data.fault_flags));
    lcd_print_line(h, 2, line);
  }

  int cur = h->menu_cursor;
  snprintf(line, 21, "%c Clear Faults", cur == 0 ? '\x00' : ' ');
  lcd_print_line(h, 3, line);
}

static void render_sysinfo(lcd_handle_t h) {
  char line[21];

  lcd_print_line(h, 0, "=== SYSTEM INFO ===");

  uint32_t up = h->data.uptime_seconds;
  snprintf(line, 21, "Up: %02luh%02lum%02lus", (unsigned long)(up / 3600),
           (unsigned long)((up % 3600) / 60), (unsigned long)(up % 60));
  lcd_print_line(h, 1, line);

  snprintf(line, 21, "FW: v1.1.0");
  lcd_print_line(h, 2, line);

  snprintf(line, 21, "Heap: %lu B", (unsigned long)esp_get_free_heap_size());
  lcd_print_line(h, 3, line);
}

static void render_edit_value(lcd_handle_t h) {
  char line[21];

  snprintf(line, 21, "Edit: %s", h->edit_label);
  lcd_print_line(h, 0, line);

  lcd_print_line(h, 1, "");

  /* Show value centered */
  if (h->edit_step >= 1.0f) {
    snprintf(line, 21, "    >>> %.0f <<<", h->edit_value);
  } else if (h->edit_step >= 0.1f) {
    snprintf(line, 21, "    >>> %.1f <<<", h->edit_value);
  } else {
    snprintf(line, 21, "   >>> %.4f <<<", h->edit_value);
  }
  lcd_print_line(h, 2, line);

  lcd_print_line(h, 3, "\x03\x04:Adj  \x02:OK");
}

/*******************************************************************************
 * Menu Logic
 ******************************************************************************/

static void emit_command(lcd_handle_t h, lcd_command_id_t cmd, float value) {
  if (h->menu_callback) {
    h->menu_callback(cmd, value, h->menu_cb_data);
  }
}

static void start_edit(lcd_handle_t h, const char *label, float current,
                       float min, float max, float step, lcd_command_id_t cmd) {
  h->edit_label = label;
  h->edit_value = current;
  h->edit_min = min;
  h->edit_max = max;
  h->edit_step = step;
  h->edit_cmd = cmd;
  h->current_screen = SCREEN_EDIT_VALUE;
}

static int get_submenu_count(menu_screen_t screen) {
  switch (screen) {
  case SCREEN_CALIBRATION:
    return CAL_ITEMS;
  case SCREEN_AUTOTUNE:
    return TUNE_ITEMS;
  case SCREEN_CONFIG:
    return CONFIG_ITEMS;
  case SCREEN_PI_GAINS:
    return PI_ITEMS;
  case SCREEN_FAULTS:
    return 1; /* Just "Clear Faults" */
  default:
    return 0;
  }
}

static void handle_menu_ok(lcd_handle_t h) {
  switch (h->current_screen) {
  case SCREEN_MENU:
    /* Enter submenu */
    h->current_screen = MENU_SCREENS[h->menu_cursor];
    h->menu_cursor = 0;
    h->menu_scroll = 0;
    break;

  case SCREEN_CALIBRATION:
    if (h->menu_cursor == 0) {
      emit_command(h, LCD_CMD_TARE, 0);
      lcd_show_message(h, "Taring...", 2000);
    } else if (h->menu_cursor == 1) {
      start_edit(h, "Cal Weight kg", 5.0f, 0.1f, 50.0f, 0.1f,
                 LCD_CMD_CALIBRATE);
    }
    break;

  case SCREEN_AUTOTUNE:
    if (h->menu_cursor == 0) {
      emit_command(h, LCD_CMD_AUTOTUNE_SPEED, 0);
      lcd_show_message(h, "Tuning speed...", 3000);
    } else if (h->menu_cursor == 1) {
      emit_command(h, LCD_CMD_AUTOTUNE_TENSION, 0);
      lcd_show_message(h, "Tuning tension...", 3000);
    }
    h->current_screen = SCREEN_MAIN;
    break;

  case SCREEN_CONFIG:
    if (h->menu_cursor == 0) {
      start_edit(h, "Encoder PPR", (float)h->data.encoder_ppr, 100, 10000, 100,
                 LCD_CMD_SET_PPR);
    } else if (h->menu_cursor == 1) {
      start_edit(h, "Max RPM", (float)h->data.max_rpm, 100, 5000, 100,
                 LCD_CMD_SET_MAX_RPM);
    } else if (h->menu_cursor == 2) {
      emit_command(h, LCD_CMD_SET_MOTOR_DIR, 0);
      lcd_show_message(h, "Dir toggled", 1500);
    } else if (h->menu_cursor == 3) {
      start_edit(h, "Tension Step kg", h->data.tension_step, 0.1f, 5.0f, 0.1f,
                 LCD_CMD_SET_TENSION_STEP);
    }
    break;

  case SCREEN_PI_GAINS:
    if (h->menu_cursor == 0) {
      start_edit(h, "Speed Kp", h->data.speed_kp, 0.0001f, 50.0f, 0.01f,
                 LCD_CMD_SET_SPEED_KP);
    } else if (h->menu_cursor == 1) {
      start_edit(h, "Speed Ki", h->data.speed_ki, 0.0001f, 20.0f, 0.001f,
                 LCD_CMD_SET_SPEED_KI);
    } else if (h->menu_cursor == 2) {
      start_edit(h, "Tension Kp", h->data.tension_kp, 0.0001f, 50.0f, 0.01f,
                 LCD_CMD_SET_TENSION_KP);
    } else if (h->menu_cursor == 3) {
      start_edit(h, "Tension Ki", h->data.tension_ki, 0.0001f, 20.0f, 0.001f,
                 LCD_CMD_SET_TENSION_KI);
    }
    break;

  case SCREEN_FAULTS:
    emit_command(h, LCD_CMD_CLEAR_FAULTS, 0);
    lcd_show_message(h, "Faults cleared", 2000);
    break;

  case SCREEN_EDIT_VALUE:
    /* Confirm edit */
    emit_command(h, h->edit_cmd, h->edit_value);
    lcd_show_message(h, "Saved!", 1500);
    h->current_screen = SCREEN_MENU;
    break;

  default:
    break;
  }
}

static void handle_navigate(lcd_handle_t h, int direction) {
  if (h->current_screen == SCREEN_EDIT_VALUE) {
    /* Adjust value */
    h->edit_value += direction * h->edit_step;
    if (h->edit_value < h->edit_min)
      h->edit_value = h->edit_min;
    if (h->edit_value > h->edit_max)
      h->edit_value = h->edit_max;
    return;
  }

  int count = 0;
  if (h->current_screen == SCREEN_MENU)
    count = MENU_ITEMS_COUNT;
  else
    count = get_submenu_count(h->current_screen);

  if (count == 0)
    return;

  h->menu_cursor += direction;
  if (h->menu_cursor < 0)
    h->menu_cursor = 0;
  if (h->menu_cursor >= count)
    h->menu_cursor = count - 1;

  /* Scroll so cursor is visible (lines 1-3, 3 visible items) */
  if (h->menu_cursor < h->menu_scroll)
    h->menu_scroll = h->menu_cursor;
  if (h->menu_cursor >= h->menu_scroll + 3)
    h->menu_scroll = h->menu_cursor - 2;
}

static void process_key_event(lcd_handle_t h, keypad_event_t ev) {
  if (ev == KEY_NONE)
    return;

  /* === MAIN SCREEN === */
  if (h->current_screen == SCREEN_MAIN) {
    switch (ev) {
    case KEY_1_SHORT:
      emit_command(h, LCD_CMD_TENSION_UP, h->data.tension_step);
      break;
    case KEY_2_SHORT:
      emit_command(h, LCD_CMD_TENSION_DOWN, h->data.tension_step);
      break;
    case KEY_1_LONG:
      /* Enter menu */
      h->current_screen = SCREEN_MENU;
      h->menu_cursor = 0;
      h->menu_scroll = 0;
      lcd_command(h, LCD_CMD_CLEAR);
      break;
    case KEY_3_HOLD:
      /* Jog left (only when IDLE) */
      if (h->data.system_state == 0) {
        emit_command(h, LCD_CMD_JOG_LEFT, 0);
      }
      break;
    case KEY_4_HOLD:
      /* Jog right (only when IDLE) */
      if (h->data.system_state == 0) {
        emit_command(h, LCD_CMD_JOG_RIGHT, 0);
      }
      break;
    case KEY_3_RELEASE:
    case KEY_4_RELEASE:
      emit_command(h, LCD_CMD_JOG_STOP, 0);
      break;
    default:
      break;
    }
    return;
  }

  /* === MENU / SUBMENU / EDIT SCREENS === */
  switch (ev) {
  case KEY_1_LONG:
    /* Exit to parent */
    if (h->current_screen == SCREEN_EDIT_VALUE) {
      /* Cancel edit — go back to menu */
      h->current_screen = SCREEN_MENU;
    } else if (h->current_screen == SCREEN_MENU) {
      /* Exit menu → main screen */
      h->current_screen = SCREEN_MAIN;
      lcd_command(h, LCD_CMD_CLEAR);
    } else {
      /* Submenu → back to menu list */
      h->current_screen = SCREEN_MENU;
      h->menu_cursor = 0;
      h->menu_scroll = 0;
    }
    break;
  case KEY_2_SHORT:
    handle_menu_ok(h);
    break;
  case KEY_3_SHORT:
    handle_navigate(h, -1); /* Up / decrease */
    break;
  case KEY_4_SHORT:
    handle_navigate(h, +1); /* Down / increase */
    break;
  case KEY_3_HOLD:
    if (h->current_screen == SCREEN_EDIT_VALUE) {
      handle_navigate(h, -1); /* Fast decrease */
    }
    break;
  case KEY_4_HOLD:
    if (h->current_screen == SCREEN_EDIT_VALUE) {
      handle_navigate(h, +1); /* Fast increase */
    }
    break;
  default:
    break;
  }
}

/*******************************************************************************
 * Public API
 ******************************************************************************/

esp_err_t lcd_init(lcd_handle_t *handle, const lcd_config_t *config) {
  if (handle == NULL || config == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  struct lcd_menu_s *lcd = calloc(1, sizeof(struct lcd_menu_s));
  if (lcd == NULL) {
    return ESP_ERR_NO_MEM;
  }

  lcd->current_screen = SCREEN_MAIN;
  lcd->data.tension_step = 0.5f; /* Default step */

  /* Initialize I2C LCD */
  esp_err_t ret = lcd_hw_init(lcd, config);
  if (ret != ESP_OK) {
    free(lcd);
    return ret;
  }

  /* Initialize keypad */
  keypad_init(lcd, config->key_gpios);

  /* Show splash screen */
  lcd_print_line(lcd, 0, " TENSION CONTROLLER ");
  lcd_print_line(lcd, 1, "    v1.1.0          ");
  lcd_print_line(lcd, 2, "   Initializing...  ");
  lcd_print_line(lcd, 3, "                    ");

  *handle = lcd;
  return ESP_OK;
}

void lcd_deinit(lcd_handle_t handle) {
  if (handle) {
    lcd_command(handle, LCD_CMD_DISPLAY_OFF);
    i2c_master_bus_rm_device(handle->i2c_dev);
    i2c_del_master_bus(handle->i2c_bus);
    free(handle);
  }
}

void lcd_update(lcd_handle_t handle, const lcd_display_data_t *data) {
  if (handle == NULL || data == NULL)
    return;

  memcpy(&handle->data, data, sizeof(lcd_display_data_t));

  switch (handle->current_screen) {
  case SCREEN_MAIN:
    render_main_screen(handle);
    break;
  case SCREEN_MENU:
    render_menu_list(handle, "MENU", MENU_ITEMS, MENU_ITEMS_COUNT);
    break;
  case SCREEN_CALIBRATION:
    render_calibration(handle);
    break;
  case SCREEN_AUTOTUNE:
    render_autotune(handle);
    break;
  case SCREEN_CONFIG:
    render_config(handle);
    break;
  case SCREEN_PI_GAINS:
    render_pi_gains(handle);
    break;
  case SCREEN_FAULTS:
    render_faults(handle);
    break;
  case SCREEN_SYSINFO:
    render_sysinfo(handle);
    break;
  case SCREEN_EDIT_VALUE:
    render_edit_value(handle);
    break;
  }
}

void lcd_process_input(lcd_handle_t handle) {
  if (handle == NULL)
    return;

  keypad_event_t ev = keypad_scan(handle);
  process_key_event(handle, ev);
}

void lcd_set_screen(lcd_handle_t handle, menu_screen_t screen) {
  if (handle) {
    handle->current_screen = screen;
    handle->menu_cursor = 0;
    handle->menu_scroll = 0;
    lcd_command(handle, LCD_CMD_CLEAR);
  }
}

menu_screen_t lcd_get_screen(lcd_handle_t handle) {
  return (handle != NULL) ? handle->current_screen : SCREEN_MAIN;
}

void lcd_set_callback(lcd_handle_t handle, lcd_command_callback_t callback,
                      void *user_data) {
  if (handle) {
    handle->old_callback = callback;
    handle->old_cb_data = user_data;
  }
}

void lcd_set_menu_callback(lcd_handle_t handle,
                           lcd_menu_command_callback_t callback,
                           void *user_data) {
  if (handle) {
    handle->menu_callback = callback;
    handle->menu_cb_data = user_data;
  }
}

void lcd_show_message(lcd_handle_t handle, const char *msg,
                      uint16_t duration_ms) {
  if (handle && msg) {
    strncpy(handle->message, msg, sizeof(handle->message) - 1);
    handle->message[sizeof(handle->message) - 1] = '\0';
    handle->message_expire_us =
        esp_timer_get_time() + (int64_t)duration_ms * 1000;
  }
}

void lcd_clear(lcd_handle_t handle) {
  if (handle) {
    lcd_command(handle, LCD_CMD_CLEAR);
  }
}
