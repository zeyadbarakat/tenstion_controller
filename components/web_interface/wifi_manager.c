/**
 * @file wifi_manager.c
 * @brief WiFi Manager Implementation
 */

#include "wifi_manager.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "mdns.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <string.h>


static const char *TAG = "wifi_mgr";

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define NVS_NAMESPACE "wifi_cfg"

struct wifi_manager_s {
  wifi_config_params_t config;
  wifi_state_t state;
  esp_netif_t *sta_netif;
  esp_netif_t *ap_netif;
  EventGroupHandle_t event_group;
  wifi_event_callback_t callback;
  void *callback_user_data;
  int retry_count;
};

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data);

wifi_config_params_t wifi_manager_get_default_config(void) {
  wifi_config_params_t config = {
      .sta_ssid = "",
      .sta_password = "",
      .ap_ssid = "TensionController",
      .ap_password = "tension123",
      .hostname = "tension-ctrl",
      .ap_channel = 1,
      .max_connections = 4,
      .fallback_to_ap = true,
      .connect_timeout_ms = 10000,
  };
  return config;
}

esp_err_t wifi_manager_init(wifi_manager_handle_t *handle,
                            const wifi_config_params_t *config) {
  if (handle == NULL || config == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  struct wifi_manager_s *mgr = calloc(1, sizeof(struct wifi_manager_s));
  if (mgr == NULL) {
    return ESP_ERR_NO_MEM;
  }

  memcpy(&mgr->config, config, sizeof(wifi_config_params_t));
  mgr->state = WIFI_STATE_DISCONNECTED;

  mgr->event_group = xEventGroupCreate();
  if (mgr->event_group == NULL) {
    free(mgr);
    return ESP_ERR_NO_MEM;
  }

  // Initialize TCP/IP stack
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  mgr->sta_netif = esp_netif_create_default_wifi_sta();
  mgr->ap_netif = esp_netif_create_default_wifi_ap();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  // Register event handlers
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                             &wifi_event_handler, mgr));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                             &wifi_event_handler, mgr));

  ESP_LOGI(TAG, "WiFi manager initialized");

  *handle = mgr;
  return ESP_OK;
}

void wifi_manager_deinit(wifi_manager_handle_t handle) {
  if (handle) {
    esp_wifi_stop();
    esp_wifi_deinit();
    vEventGroupDelete(handle->event_group);
    free(handle);
  }
}

esp_err_t wifi_manager_start_sta(wifi_manager_handle_t handle) {
  if (handle == NULL)
    return ESP_ERR_INVALID_ARG;

  if (strlen(handle->config.sta_ssid) == 0) {
    ESP_LOGW(TAG, "No STA SSID configured, starting AP mode");
    return wifi_manager_start_ap(handle);
  }

  ESP_LOGI(TAG, "Connecting to WiFi: %s", handle->config.sta_ssid);
  handle->state = WIFI_STATE_CONNECTING;
  handle->retry_count = 0;

  wifi_config_t wifi_config = {0};
  strncpy((char *)wifi_config.sta.ssid, handle->config.sta_ssid,
          sizeof(wifi_config.sta.ssid));
  strncpy((char *)wifi_config.sta.password, handle->config.sta_password,
          sizeof(wifi_config.sta.password));

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  // Wait for connection
  EventBits_t bits = xEventGroupWaitBits(
      handle->event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE,
      pdMS_TO_TICKS(handle->config.connect_timeout_ms));

  if (bits & WIFI_CONNECTED_BIT) {
    ESP_LOGI(TAG, "Connected to WiFi");
    handle->state = WIFI_STATE_CONNECTED;

    // Start mDNS
    mdns_init();
    mdns_hostname_set(handle->config.hostname);
    mdns_instance_name_set("Tension Controller");

    return ESP_OK;
  }

  if (handle->config.fallback_to_ap) {
    ESP_LOGW(TAG, "STA connection failed, falling back to AP");
    return wifi_manager_start_ap(handle);
  }

  handle->state = WIFI_STATE_ERROR;
  return ESP_FAIL;
}

esp_err_t wifi_manager_start_ap(wifi_manager_handle_t handle) {
  if (handle == NULL)
    return ESP_ERR_INVALID_ARG;

  ESP_LOGI(TAG, "Starting AP mode: %s", handle->config.ap_ssid);

  wifi_config_t wifi_config = {
      .ap =
          {
              .channel = handle->config.ap_channel,
              .max_connection = handle->config.max_connections,
              .authmode = WIFI_AUTH_WPA2_PSK,
          },
  };

  strncpy((char *)wifi_config.ap.ssid, handle->config.ap_ssid,
          sizeof(wifi_config.ap.ssid));
  wifi_config.ap.ssid_len = strlen(handle->config.ap_ssid);
  strncpy((char *)wifi_config.ap.password, handle->config.ap_password,
          sizeof(wifi_config.ap.password));

  if (strlen(handle->config.ap_password) < 8) {
    wifi_config.ap.authmode = WIFI_AUTH_OPEN;
  }

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  handle->state = WIFI_STATE_AP_MODE;

  // Start mDNS
  mdns_init();
  mdns_hostname_set(handle->config.hostname);

  ESP_LOGI(TAG, "AP started. Connect to: %s, IP: 192.168.4.1",
           handle->config.ap_ssid);

  return ESP_OK;
}

esp_err_t wifi_manager_stop(wifi_manager_handle_t handle) {
  if (handle == NULL)
    return ESP_ERR_INVALID_ARG;

  esp_wifi_stop();
  handle->state = WIFI_STATE_DISCONNECTED;

  return ESP_OK;
}

wifi_state_t wifi_manager_get_state(wifi_manager_handle_t handle) {
  return (handle != NULL) ? handle->state : WIFI_STATE_DISCONNECTED;
}

esp_err_t wifi_manager_get_ip(wifi_manager_handle_t handle, char *ip_str,
                              size_t len) {
  if (handle == NULL || ip_str == NULL)
    return ESP_ERR_INVALID_ARG;

  esp_netif_ip_info_t ip_info;
  esp_netif_t *netif = (handle->state == WIFI_STATE_AP_MODE)
                           ? handle->ap_netif
                           : handle->sta_netif;

  if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
    snprintf(ip_str, len, IPSTR, IP2STR(&ip_info.ip));
    return ESP_OK;
  }

  return ESP_FAIL;
}

void wifi_manager_set_callback(wifi_manager_handle_t handle,
                               wifi_event_callback_t cb, void *user_data) {
  if (handle) {
    handle->callback = cb;
    handle->callback_user_data = user_data;
  }
}

esp_err_t wifi_manager_save_config(wifi_manager_handle_t handle) {
  if (handle == NULL)
    return ESP_ERR_INVALID_ARG;

  nvs_handle_t nvs;
  esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
  if (ret != ESP_OK)
    return ret;

  nvs_set_str(nvs, "sta_ssid", handle->config.sta_ssid);
  nvs_set_str(nvs, "sta_pass", handle->config.sta_password);
  nvs_set_str(nvs, "ap_ssid", handle->config.ap_ssid);
  nvs_set_str(nvs, "ap_pass", handle->config.ap_password);

  ret = nvs_commit(nvs);
  nvs_close(nvs);

  ESP_LOGI(TAG, "WiFi config saved");
  return ret;
}

esp_err_t wifi_manager_load_config(wifi_manager_handle_t handle) {
  if (handle == NULL)
    return ESP_ERR_INVALID_ARG;

  nvs_handle_t nvs;
  esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs);
  if (ret != ESP_OK)
    return ret;

  size_t len;

  len = sizeof(handle->config.sta_ssid);
  nvs_get_str(nvs, "sta_ssid", handle->config.sta_ssid, &len);

  len = sizeof(handle->config.sta_password);
  nvs_get_str(nvs, "sta_pass", handle->config.sta_password, &len);

  nvs_close(nvs);

  ESP_LOGI(TAG, "WiFi config loaded");
  return ESP_OK;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  struct wifi_manager_s *mgr = (struct wifi_manager_s *)arg;

  if (event_base == WIFI_EVENT) {
    switch (event_id) {
    case WIFI_EVENT_STA_START:
      esp_wifi_connect();
      break;

    case WIFI_EVENT_STA_DISCONNECTED:
      if (mgr->retry_count < 5) {
        esp_wifi_connect();
        mgr->retry_count++;
        ESP_LOGI(TAG, "Retry connecting... (%d/5)", mgr->retry_count);
      } else {
        xEventGroupSetBits(mgr->event_group, WIFI_FAIL_BIT);
        mgr->state = WIFI_STATE_DISCONNECTED;
      }
      break;

    case WIFI_EVENT_AP_STACONNECTED: {
      wifi_event_ap_staconnected_t *event =
          (wifi_event_ap_staconnected_t *)event_data;
      ESP_LOGI(TAG, "Client connected, AID=%d", event->aid);
      break;
    }

    case WIFI_EVENT_AP_STADISCONNECTED: {
      wifi_event_ap_stadisconnected_t *event =
          (wifi_event_ap_stadisconnected_t *)event_data;
      ESP_LOGI(TAG, "Client disconnected, AID=%d", event->aid);
      break;
    }
    }
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    mgr->retry_count = 0;
    xEventGroupSetBits(mgr->event_group, WIFI_CONNECTED_BIT);
    mgr->state = WIFI_STATE_CONNECTED;

    if (mgr->callback) {
      mgr->callback(WIFI_STATE_CONNECTED, mgr->callback_user_data);
    }
  }
}
