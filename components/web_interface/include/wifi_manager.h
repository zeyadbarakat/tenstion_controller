/**
 * @file wifi_manager.h
 * @brief WiFi Manager with Station and AP mode support
 */

#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  WIFI_STATE_DISCONNECTED = 0,
  WIFI_STATE_CONNECTING,
  WIFI_STATE_CONNECTED,
  WIFI_STATE_AP_MODE,
  WIFI_STATE_ERROR
} wifi_state_t;

typedef struct {
  char sta_ssid[32];
  char sta_password[64];
  char ap_ssid[32];
  char ap_password[64];
  char hostname[32];
  uint8_t ap_channel;
  uint8_t max_connections;
  bool fallback_to_ap;
  uint32_t connect_timeout_ms;
} wifi_config_params_t;

typedef void (*wifi_event_callback_t)(wifi_state_t state, void *user_data);

typedef struct wifi_manager_s *wifi_manager_handle_t;

esp_err_t wifi_manager_init(wifi_manager_handle_t *handle,
                            const wifi_config_params_t *config);
void wifi_manager_deinit(wifi_manager_handle_t handle);
esp_err_t wifi_manager_start_sta(wifi_manager_handle_t handle);
esp_err_t wifi_manager_start_ap(wifi_manager_handle_t handle);
esp_err_t wifi_manager_stop(wifi_manager_handle_t handle);
wifi_state_t wifi_manager_get_state(wifi_manager_handle_t handle);
esp_err_t wifi_manager_get_ip(wifi_manager_handle_t handle, char *ip_str,
                              size_t len);
void wifi_manager_set_callback(wifi_manager_handle_t handle,
                               wifi_event_callback_t cb, void *user_data);
esp_err_t wifi_manager_save_config(wifi_manager_handle_t handle);
esp_err_t wifi_manager_load_config(wifi_manager_handle_t handle);
wifi_config_params_t wifi_manager_get_default_config(void);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_MANAGER_H */
