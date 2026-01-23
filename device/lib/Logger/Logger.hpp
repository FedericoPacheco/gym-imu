#pragma once
#include "esp_log.h"
#include <atomic>
#include <cstdint>

enum class LogLevel { ERROR, WARN, INFO, DEBUG };

class Logger {
private:
  inline static constexpr const char *tag = "GymIMU";
  static const int MAX_LOG_MESSAGE_LENGTH = 512;

  LogLevel currentLevel;
  std::atomic<uint64_t> sequenceNumber;

public:
  Logger(LogLevel level = LogLevel::INFO);
  ~Logger() = default;

  void setLevel(LogLevel level);

  void error(const char *format, ...);
  void warn(const char *format, ...);
  void info(const char *format, ...);
  void debug(const char *format, ...);

private:
  void log(LogLevel level, const char *formattedMessageStr,
           va_list messageArgs);
  void formatTime(int64_t upTimeInMicros, char *timeBuffer);
  esp_log_level_t mapToEspLogLevel(LogLevel level);
};