#pragma once

// Mock ESPHome log header for unit testing

#include <cstdarg>
#include <cstdio>
#include <cstdint>

namespace esphome
{

  enum LogLevel
  {
    ESPHOME_LOG_LEVEL_NONE = 0,
    ESPHOME_LOG_LEVEL_ERROR = 1,
    ESPHOME_LOG_LEVEL_WARN = 2,
    ESPHOME_LOG_LEVEL_INFO = 3,
    ESPHOME_LOG_LEVEL_CONFIG = 4,
    ESPHOME_LOG_LEVEL_DEBUG = 5,
    ESPHOME_LOG_LEVEL_VERBOSE = 6,
    ESPHOME_LOG_LEVEL_VERY_VERBOSE = 7
  };

  // Mock logging functions - do nothing in unit tests
  inline void esp_log_printf_(int level, const char *tag, int line, const char *format, ...)
  {
    // Disabled for unit tests
  }

} // namespace esphome

// Mock logging macros
#define ESP_LOGD(tag, ...) ((void)0)
#define ESP_LOGI(tag, ...) ((void)0)
#define ESP_LOGW(tag, ...) ((void)0)
#define ESP_LOGE(tag, ...) ((void)0)
#define ESP_LOGCONFIG(tag, ...) ((void)0)
#define ESP_LOGV(tag, ...) ((void)0)
#define ESP_LOGVV(tag, ...) ((void)0)

// LOG_STEPPER macro for stepper components
#define LOG_STEPPER(obj) ((void)0)
