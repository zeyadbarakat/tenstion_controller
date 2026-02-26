/**
 * @file web_server.c
 * @brief HTTP Server Implementation with REST API and WebSocket
 *
 * Supports both HTTP polling (/api/status) and WebSocket push (/ws).
 * WebSocket provides real-time updates at 50Hz for smooth charts.
 */

#include "web_server.h"
#include "cJSON.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "web_srv";

// Maximum WebSocket clients
#define MAX_WS_CLIENTS 4

// Embedded HTML file (from CMakeLists.txt EMBED_FILES)
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_index_html_end");

// WebSocket client tracking
typedef struct {
  int fd;      // Socket file descriptor (-1 = unused)
  bool active; // Connection is active
} ws_client_t;

struct web_server_s {
  httpd_handle_t server;
  uint16_t port;
  web_command_callback_t cmd_callback;
  void *cmd_user_data;
  web_status_callback_t status_callback;
  void *status_user_data;
  // WebSocket clients
  ws_client_t ws_clients[MAX_WS_CLIENTS];
  atomic_int ws_client_count;
};

// Forward declarations
static esp_err_t index_handler(httpd_req_t *req);
static esp_err_t api_status_handler(httpd_req_t *req);
static esp_err_t api_command_handler(httpd_req_t *req);
static esp_err_t api_config_get_handler(httpd_req_t *req);
static esp_err_t api_config_post_handler(httpd_req_t *req);
static esp_err_t ws_handler(httpd_req_t *req);

// WebSocket client management
static void ws_client_add(struct web_server_s *srv, int fd);
static void ws_client_remove(struct web_server_s *srv, int fd);

esp_err_t web_server_init(web_server_handle_t *handle, uint16_t port) {
  if (handle == NULL)
    return ESP_ERR_INVALID_ARG;

  struct web_server_s *srv = calloc(1, sizeof(struct web_server_s));
  if (srv == NULL)
    return ESP_ERR_NO_MEM;

  srv->port = port;

  // Initialize WebSocket client slots
  for (int i = 0; i < MAX_WS_CLIENTS; i++) {
    srv->ws_clients[i].fd = -1;
    srv->ws_clients[i].active = false;
  }
  atomic_init(&srv->ws_client_count, 0);

  ESP_LOGI(TAG, "Web server initialized on port %d", port);

  *handle = srv;
  return ESP_OK;
}

void web_server_deinit(web_server_handle_t handle) {
  if (handle) {
    web_server_stop(handle);
    free(handle);
  }
}

esp_err_t web_server_start(web_server_handle_t handle) {
  if (handle == NULL)
    return ESP_ERR_INVALID_ARG;

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = handle->port;
  config.max_uri_handlers = 10; // Increased for WebSocket
  config.lru_purge_enable = true;
  config.recv_wait_timeout = 10;
  config.send_wait_timeout = 10;

  esp_err_t ret = httpd_start(&handle->server, &config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start server: %s", esp_err_to_name(ret));
    return ret;
  }

  // Register URI handlers
  httpd_uri_t index_uri = {.uri = "/",
                           .method = HTTP_GET,
                           .handler = index_handler,
                           .user_ctx = handle};
  httpd_register_uri_handler(handle->server, &index_uri);

  httpd_uri_t api_status_uri = {.uri = "/api/status",
                                .method = HTTP_GET,
                                .handler = api_status_handler,
                                .user_ctx = handle};
  httpd_register_uri_handler(handle->server, &api_status_uri);

  httpd_uri_t api_cmd_uri = {.uri = "/api/command",
                             .method = HTTP_POST,
                             .handler = api_command_handler,
                             .user_ctx = handle};
  httpd_register_uri_handler(handle->server, &api_cmd_uri);

  // Config API endpoints (NVS storage)
  httpd_uri_t api_config_get = {.uri = "/api/config",
                                .method = HTTP_GET,
                                .handler = api_config_get_handler,
                                .user_ctx = handle};
  httpd_register_uri_handler(handle->server, &api_config_get);

  httpd_uri_t api_config_post = {.uri = "/api/config",
                                 .method = HTTP_POST,
                                 .handler = api_config_post_handler,
                                 .user_ctx = handle};
  httpd_register_uri_handler(handle->server, &api_config_post);

  // WebSocket endpoint for real-time updates
  httpd_uri_t ws_uri = {.uri = "/ws",
                        .method = HTTP_GET,
                        .handler = ws_handler,
                        .user_ctx = handle,
                        .is_websocket = true,
                        .handle_ws_control_frames = true};
  httpd_register_uri_handler(handle->server, &ws_uri);

  ESP_LOGI(TAG, "Web server started on port %d (WebSocket enabled)",
           handle->port);
  return ESP_OK;
}

esp_err_t web_server_stop(web_server_handle_t handle) {
  if (handle && handle->server) {
    httpd_stop(handle->server);
    handle->server = NULL;
    ESP_LOGI(TAG, "Web server stopped");
  }
  return ESP_OK;
}

void web_server_set_command_callback(web_server_handle_t handle,
                                     web_command_callback_t cb,
                                     void *user_data) {
  if (handle) {
    handle->cmd_callback = cb;
    handle->cmd_user_data = user_data;
  }
}

void web_server_set_status_callback(web_server_handle_t handle,
                                    web_status_callback_t cb, void *user_data) {
  if (handle) {
    handle->status_callback = cb;
    handle->status_user_data = user_data;
  }
}

esp_err_t web_server_broadcast_status(web_server_handle_t handle,
                                      const web_status_data_t *status) {
  // HTTP polling mode - status is fetched via /api/status
  (void)handle;
  (void)status;
  return ESP_OK;
}

// === HTTP Handlers ===

static esp_err_t index_handler(httpd_req_t *req) {
  ESP_LOGI(TAG, "Serving index.html");
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  size_t len = index_html_end - index_html_start;
  httpd_resp_send(req, (const char *)index_html_start, len);
  return ESP_OK;
}

static esp_err_t api_status_handler(httpd_req_t *req) {
  web_server_handle_t srv = (web_server_handle_t)req->user_ctx;

  web_status_data_t status = {0};
  if (srv->status_callback) {
    srv->status_callback(&status, srv->status_user_data);
  }

  cJSON *root = cJSON_CreateObject();
  if (root == NULL) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  cJSON_AddNumberToObject(root, "tension", status.tension_kg);
  cJSON_AddNumberToObject(root, "tensionSP", status.tension_setpoint);
  cJSON_AddNumberToObject(root, "speed", status.speed_rpm);
  cJSON_AddNumberToObject(root, "pwm", status.pwm_percent);
  cJSON_AddNumberToObject(root, "state", status.system_state);
  cJSON_AddNumberToObject(root, "faults", status.fault_flags);
  cJSON_AddBoolToObject(root, "calibrated", status.calibrated);
  cJSON_AddNumberToObject(root, "uptime", status.uptime_seconds);
  cJSON_AddNumberToObject(root, "heap", esp_get_free_heap_size());

  char *json = cJSON_PrintUnformatted(root);
  cJSON_Delete(root);

  if (json == NULL) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_send(req, json, strlen(json));
  free(json);

  return ESP_OK;
}

static esp_err_t api_command_handler(httpd_req_t *req) {
  web_server_handle_t srv = (web_server_handle_t)req->user_ctx;

  char buf[128];
  int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
  if (ret <= 0) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No data");
    return ESP_FAIL;
  }
  buf[ret] = '\0';

  ESP_LOGI(TAG, "Command received: %s", buf);

  cJSON *root = cJSON_Parse(buf);
  if (root == NULL) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
    return ESP_FAIL;
  }

  cJSON *cmd_json = cJSON_GetObjectItem(root, "cmd");
  cJSON *val_json = cJSON_GetObjectItem(root, "value");

  if (cmd_json && srv->cmd_callback) {
    web_command_t cmd = (web_command_t)cmd_json->valueint;
    float value = val_json ? (float)val_json->valuedouble : 0.0f;
    ESP_LOGI(TAG, "Executing command %d with value %.2f", cmd, value);
    srv->cmd_callback(cmd, value, srv->cmd_user_data);
  }

  cJSON_Delete(root);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_send(req, "{\"ok\":true}", -1);

  return ESP_OK;
}

// Config API - Save/Load settings from NVS
static esp_err_t api_config_get_handler(httpd_req_t *req) {
  nvs_handle_t nvs;
  esp_err_t err = nvs_open("config", NVS_READONLY, &nvs);

  cJSON *root = cJSON_CreateObject();

  if (err == ESP_OK) {
    uint16_t ppr = 600;
    int32_t cal_offset = 0;
    int32_t cal_scale_x1000 = 1000; // 0.001 * 1000000
    int32_t speed_kp_x = 5000;
    int32_t speed_ki_x = 1000;
    int32_t tension_kp_x = 2000;
    int32_t tension_ki_x = 500;

    nvs_get_u16(nvs, "ppr", &ppr);
    nvs_get_i32(nvs, "cal_offset", &cal_offset);
    nvs_get_i32(nvs, "cal_scale", &cal_scale_x1000);
    nvs_get_i32(nvs, "speed_kp", &speed_kp_x);
    nvs_get_i32(nvs, "speed_ki", &speed_ki_x);
    nvs_get_i32(nvs, "tension_kp", &tension_kp_x);
    nvs_get_i32(nvs, "tension_ki", &tension_ki_x);
    nvs_close(nvs);

    cJSON_AddNumberToObject(root, "ppr", ppr);
    cJSON_AddNumberToObject(root, "cal_offset", cal_offset);
    cJSON_AddNumberToObject(root, "cal_scale",
                            (float)cal_scale_x1000 / 1000000.0f);
    cJSON_AddNumberToObject(root, "speed_kp", (float)speed_kp_x / 10000.0f);
    cJSON_AddNumberToObject(root, "speed_ki", (float)speed_ki_x / 10000.0f);
    cJSON_AddNumberToObject(root, "tension_kp", (float)tension_kp_x / 10000.0f);
    cJSON_AddNumberToObject(root, "tension_ki", (float)tension_ki_x / 10000.0f);
    cJSON_AddBoolToObject(root, "loaded", true);
  } else {
    // Defaults
    cJSON_AddNumberToObject(root, "ppr", 600);
    cJSON_AddNumberToObject(root, "cal_offset", 0);
    cJSON_AddNumberToObject(root, "cal_scale", 0.001);
    cJSON_AddNumberToObject(root, "speed_kp", 1.0);
    cJSON_AddNumberToObject(root, "speed_ki", 0.1);
    cJSON_AddNumberToObject(root, "tension_kp", 2.0);
    cJSON_AddNumberToObject(root, "tension_ki", 0.05);
    cJSON_AddBoolToObject(root, "loaded", false);
  }

  char *json = cJSON_PrintUnformatted(root);
  cJSON_Delete(root);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_send(req, json, strlen(json));
  free(json);
  return ESP_OK;
}

static esp_err_t api_config_post_handler(httpd_req_t *req) {
  char buf[256];
  int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
  if (ret <= 0) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No data");
    return ESP_FAIL;
  }
  buf[ret] = '\0';

  ESP_LOGI(TAG, "Config save: %s", buf);

  cJSON *root = cJSON_Parse(buf);
  if (root == NULL) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
    return ESP_FAIL;
  }

  nvs_handle_t nvs;
  esp_err_t err = nvs_open("config", NVS_READWRITE, &nvs);
  if (err != ESP_OK) {
    cJSON_Delete(root);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                        "NVS open failed");
    return ESP_FAIL;
  }

  cJSON *item;
  if ((item = cJSON_GetObjectItem(root, "ppr")) != NULL) {
    nvs_set_u16(nvs, "ppr", (uint16_t)item->valueint);
    ESP_LOGI(TAG, "Saved PPR: %d", item->valueint);
  }
  if ((item = cJSON_GetObjectItem(root, "cal_offset")) != NULL) {
    nvs_set_i32(nvs, "cal_offset", (int32_t)item->valueint);
    ESP_LOGI(TAG, "Saved cal_offset: %d", item->valueint);
  }
  if ((item = cJSON_GetObjectItem(root, "cal_scale")) != NULL) {
    nvs_set_i32(nvs, "cal_scale", (int32_t)(item->valuedouble * 1000000.0));
    ESP_LOGI(TAG, "Saved cal_scale: %f", item->valuedouble);
  }
  if ((item = cJSON_GetObjectItem(root, "speed_kp")) != NULL) {
    nvs_set_i32(nvs, "speed_kp", (int32_t)(item->valuedouble * 10000.0));
    ESP_LOGI(TAG, "Saved speed_kp: %f", item->valuedouble);
  }
  if ((item = cJSON_GetObjectItem(root, "speed_ki")) != NULL) {
    nvs_set_i32(nvs, "speed_ki", (int32_t)(item->valuedouble * 10000.0));
    ESP_LOGI(TAG, "Saved speed_ki: %f", item->valuedouble);
  }
  if ((item = cJSON_GetObjectItem(root, "tension_kp")) != NULL) {
    nvs_set_i32(nvs, "tension_kp", (int32_t)(item->valuedouble * 10000.0));
    ESP_LOGI(TAG, "Saved tension_kp: %f", item->valuedouble);
  }
  if ((item = cJSON_GetObjectItem(root, "tension_ki")) != NULL) {
    nvs_set_i32(nvs, "tension_ki", (int32_t)(item->valuedouble * 10000.0));
    ESP_LOGI(TAG, "Saved tension_ki: %f", item->valuedouble);
  }

  nvs_commit(nvs);
  nvs_close(nvs);
  cJSON_Delete(root);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_send(req, "{\"ok\":true,\"saved\":true}", -1);
  return ESP_OK;
}

// === WebSocket Implementation ===

// Add a WebSocket client
static void ws_client_add(struct web_server_s *srv, int fd) {
  for (int i = 0; i < MAX_WS_CLIENTS; i++) {
    if (!srv->ws_clients[i].active) {
      srv->ws_clients[i].fd = fd;
      srv->ws_clients[i].active = true;
      atomic_fetch_add(&srv->ws_client_count, 1);
      ESP_LOGI(TAG, "WebSocket client connected (fd=%d, total=%d)", fd,
               atomic_load(&srv->ws_client_count));
      return;
    }
  }
  ESP_LOGW(TAG, "WebSocket client rejected - max clients reached");
}

// Remove a WebSocket client
static void ws_client_remove(struct web_server_s *srv, int fd) {
  for (int i = 0; i < MAX_WS_CLIENTS; i++) {
    if (srv->ws_clients[i].fd == fd && srv->ws_clients[i].active) {
      srv->ws_clients[i].fd = -1;
      srv->ws_clients[i].active = false;
      atomic_fetch_sub(&srv->ws_client_count, 1);
      ESP_LOGI(TAG, "WebSocket client disconnected (fd=%d, total=%d)", fd,
               atomic_load(&srv->ws_client_count));
      return;
    }
  }
}

// WebSocket handler
static esp_err_t ws_handler(httpd_req_t *req) {
  struct web_server_s *srv = (struct web_server_s *)req->user_ctx;

  if (req->method == HTTP_GET) {
    // This is the initial handshake - register client immediately
    int fd = httpd_req_to_sockfd(req);
    ESP_LOGI(TAG, "WebSocket handshake from fd=%d", fd);
    ws_client_add(srv, fd);
    return ESP_OK;
  }

  // Handle WebSocket frame
  httpd_ws_frame_t ws_pkt;
  memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
  ws_pkt.type = HTTPD_WS_TYPE_TEXT;

  // Get frame info
  esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "httpd_ws_recv_frame failed: %d", ret);
    return ret;
  }

  // Handle different frame types
  if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE) {
    ws_client_remove(srv, httpd_req_to_sockfd(req));
    return ESP_OK;
  }

  // If it's a new connection (first message or PONG after PING)
  int fd = httpd_req_to_sockfd(req);
  bool found = false;
  for (int i = 0; i < MAX_WS_CLIENTS; i++) {
    if (srv->ws_clients[i].fd == fd) {
      found = true;
      break;
    }
  }
  if (!found) {
    ws_client_add(srv, fd);
  }

  // For now, we don't process incoming WebSocket messages
  // The broadcast is done separately via web_server_ws_broadcast

  return ESP_OK;
}

// Broadcast JSON data to all connected WebSocket clients
esp_err_t web_server_ws_broadcast(web_server_handle_t handle,
                                  const char *json_data) {
  if (handle == NULL || handle->server == NULL || json_data == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (atomic_load(&handle->ws_client_count) == 0) {
    return ESP_OK; // No clients connected
  }

  httpd_ws_frame_t ws_pkt;
  memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
  ws_pkt.type = HTTPD_WS_TYPE_TEXT;
  ws_pkt.payload = (uint8_t *)json_data;
  ws_pkt.len = strlen(json_data);

  // Send to all connected clients
  for (int i = 0; i < MAX_WS_CLIENTS; i++) {
    if (handle->ws_clients[i].active) {
      int fd = handle->ws_clients[i].fd;
      esp_err_t ret = httpd_ws_send_frame_async(handle->server, fd, &ws_pkt);
      if (ret != ESP_OK) {
        // Client disconnected, remove from list
        ESP_LOGD(TAG, "Failed to send to fd=%d, removing", fd);
        ws_client_remove(handle, fd);
      }
    }
  }

  return ESP_OK;
}

// Get number of connected WebSocket clients
int web_server_ws_client_count(web_server_handle_t handle) {
  if (handle == NULL)
    return 0;
  return atomic_load(&handle->ws_client_count);
}
