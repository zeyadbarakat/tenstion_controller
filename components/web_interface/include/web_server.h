/**
 * @file web_server.h
 * @brief HTTP Server with WebSocket Support
 */

#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "esp_err.h"
#include "esp_http_server.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float tension_kg;
  float tension_setpoint;
  float speed_rpm;
  float speed_setpoint;
  float pwm_percent;
  uint8_t system_state;
  uint16_t fault_flags;
  bool calibrated;
  uint32_t uptime_seconds;
} web_status_data_t;

typedef enum {
  WEB_CMD_START = 0,
  WEB_CMD_STOP,
  WEB_CMD_ESTOP,
  WEB_CMD_SET_TENSION,
  WEB_CMD_SET_SPEED,
  WEB_CMD_TARE,
  WEB_CMD_CALIBRATE,
  WEB_CMD_AUTOTUNE_SPEED,
  WEB_CMD_AUTOTUNE_TENSION,
  WEB_CMD_RESET_FAULTS,
  WEB_CMD_SAVE_CONFIG,
  // Jog mode commands (when stopped)
  WEB_CMD_JOG_START = 20, // value: -1 = left, +1 = right
  WEB_CMD_JOG_STOP = 21
} web_command_t;

typedef void (*web_command_callback_t)(web_command_t cmd, float value,
                                       void *user_data);
typedef void (*web_status_callback_t)(web_status_data_t *status,
                                      void *user_data);

typedef struct web_server_s *web_server_handle_t;

esp_err_t web_server_init(web_server_handle_t *handle, uint16_t port);
void web_server_deinit(web_server_handle_t handle);
esp_err_t web_server_start(web_server_handle_t handle);
esp_err_t web_server_stop(web_server_handle_t handle);
void web_server_set_command_callback(web_server_handle_t handle,
                                     web_command_callback_t cb,
                                     void *user_data);
void web_server_set_status_callback(web_server_handle_t handle,
                                    web_status_callback_t cb, void *user_data);
esp_err_t web_server_broadcast_status(web_server_handle_t handle,
                                      const web_status_data_t *status);

// WebSocket real-time broadcast
esp_err_t web_server_ws_broadcast(web_server_handle_t handle,
                                  const char *json_data);
int web_server_ws_client_count(web_server_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* WEB_SERVER_H */
