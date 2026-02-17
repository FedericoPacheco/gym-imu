#include "esp_log.h"
#include "esp_timer.h"
#include <Logger.hpp>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <stdio.h>
#include <sys/param.h>
#include <sys/stat.h>

Logger::Logger(LogLevel level) {
  this->sequenceNumber = 0;
  this->setLevel(level);
}

void Logger::log(LogLevel level, const char *formattedMessageStr,
                 va_list messageArgs) {
  uint64_t incrementedSequence =
      sequenceNumber.fetch_add(1, std::memory_order_relaxed);

  char humanReadableUpTime[32];
  this->getHumanReadableUpTime(humanReadableUpTime);

  char finalStrWithPrefix[Logger::MAX_LOG_MESSAGE_LENGTH];
  snprintf(finalStrWithPrefix, sizeof(finalStrWithPrefix),
           "[%s] [t=%s] [seq=%llu] %s\n", this->mapLevelToString(level),
           humanReadableUpTime, incrementedSequence, formattedMessageStr);

  esp_log_level_t espLevel = this->mapToEspLogLevel(level);

  esp_log_writev(espLevel, tag, finalStrWithPrefix, messageArgs);
}
void Logger::getHumanReadableUpTime(char *timerBuffer) {
  const uint64_t ONE_SECOND_IN_MICROS = 1000000;
  const uint64_t ONE_MINUTE_IN_MICROS = ONE_SECOND_IN_MICROS * 60;
  const uint64_t ONE_HOUR_IN_MICROS = ONE_MINUTE_IN_MICROS * 60;

  uint64_t upTimeInMicros = static_cast<uint64_t>(esp_timer_get_time());

  uint64_t hours = upTimeInMicros / ONE_HOUR_IN_MICROS;
  uint64_t remainder = upTimeInMicros % ONE_HOUR_IN_MICROS;

  uint64_t minutes = remainder / ONE_MINUTE_IN_MICROS;
  remainder = remainder % ONE_MINUTE_IN_MICROS;

  uint64_t seconds = remainder / ONE_SECOND_IN_MICROS;
  remainder = remainder % ONE_SECOND_IN_MICROS;

  uint64_t millis = remainder / 1000;

  // Format: HH:MM:SS.mmm e.g. "01:23:45.678"
  snprintf(timerBuffer, 32, "%02llu:%02llu:%02llu.%03llu", hours, minutes,
           seconds, millis);
}
esp_log_level_t Logger::mapToEspLogLevel(LogLevel level) {
  switch (level) {
  case LogLevel::ERROR:
    return ESP_LOG_ERROR;
  case LogLevel::WARN:
    return ESP_LOG_WARN;
  case LogLevel::INFO:
    return ESP_LOG_INFO;
  case LogLevel::DEBUG:
    return ESP_LOG_DEBUG;
  default:
    return ESP_LOG_INFO;
  }
}
const char *Logger::mapLevelToString(LogLevel level) {
  switch (level) {
  case LogLevel::ERROR:
    return " ERROR ";
  case LogLevel::WARN:
    return " WARN  ";
  case LogLevel::INFO:
    return " INFO  ";
  case LogLevel::DEBUG:
    return " DEBUG ";
  default:
    return " INFO  ";
  }
}

void Logger::error(const char *formattedMessageStr, ...) {
  if (currentLevel < LogLevel::ERROR)
    return;

  va_list messageArgs;
  va_start(messageArgs, formattedMessageStr);
  this->log(LogLevel::ERROR, formattedMessageStr, messageArgs);
  va_end(messageArgs);
}
void Logger::warn(const char *formattedMessageStr, ...) {
  if (currentLevel < LogLevel::WARN)
    return;

  va_list messageArgs;
  va_start(messageArgs, formattedMessageStr);
  this->log(LogLevel::WARN, formattedMessageStr, messageArgs);
  va_end(messageArgs);
}
void Logger::info(const char *formattedMessageStr, ...) {
  if (currentLevel < LogLevel::INFO)
    return;

  va_list messageArgs;
  va_start(messageArgs, formattedMessageStr);
  this->log(LogLevel::INFO, formattedMessageStr, messageArgs);
  va_end(messageArgs);
}
void Logger::debug(const char *formattedMessageStr, ...) {
  if (currentLevel < LogLevel::DEBUG)
    return;

  va_list messageArgs;
  va_start(messageArgs, formattedMessageStr);
  this->log(LogLevel::DEBUG, formattedMessageStr, messageArgs);
  va_end(messageArgs);
}

void Logger::setLevel(LogLevel level) {
  esp_log_level_set(tag, mapToEspLogLevel(level));
  this->currentLevel = level;
}
