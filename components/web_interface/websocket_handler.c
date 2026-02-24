/**
 * @file websocket_handler.c
 * @brief WebSocket Handler for Real-time Updates
 */

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "web_server.h"


static const char *TAG = "ws_handler";

// This file is a placeholder - the WebSocket handling is integrated
// into web_server.c. This file can be used for additional WebSocket
// features like binary protocol or compression in the future.

void websocket_task(void *arg) {
  // Placeholder for dedicated WebSocket broadcast task
  // The current implementation broadcasts in web_server_broadcast_status
  vTaskDelete(NULL);
}
