/**
 * @file web_pages.c
 * @brief Web Page Generation Helpers
 */

#include "web_server.h"
#include <string.h>

// This file provides helper functions for generating dynamic content.
// The main HTML/CSS/JS files are embedded from the www/ directory.

const char *web_state_to_string(uint8_t state) {
  switch (state) {
  case 0:
    return "Idle";
  case 1:
    return "Starting";
  case 2:
    return "Running";
  case 3:
    return "Stopping";
  case 4:
    return "Warning";
  case 5:
    return "Fault";
  case 6:
    return "E-Stop";
  default:
    return "Unknown";
  }
}

const char *web_fault_to_string(uint16_t fault) {
  if (fault & 0x0001)
    return "Tension Over Limit";
  if (fault & 0x0002)
    return "Speed Over Limit";
  if (fault & 0x0004)
    return "Motor Stall";
  if (fault & 0x0008)
    return "Encoder Failure";
  if (fault & 0x0010)
    return "Load Cell Failure";
  if (fault & 0x0020)
    return "Watchdog Timeout";
  if (fault & 0x8000)
    return "Emergency Stop";
  return "None";
}
