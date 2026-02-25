#include <Logger.hpp>

UARTLogger::UARTLogger(const char *tag, LogLevel level) {
  snprintf(this->tag, UARTLogger::TAG_MAX_LENGTH, "%s", tag);
  this->setLevel(level);
}

void UARTLogger::log(LogLevel level, const char *formattedMessageStr,
                     va_list messageArgs) {
  char humanReadableUpTime[32];
  this->getHumanReadableUpTime(humanReadableUpTime);

  char finalStrWithPrefix[UARTLogger::MAX_LOG_MESSAGE_LENGTH];
  snprintf(finalStrWithPrefix, sizeof(finalStrWithPrefix),
           "[ %s ] [ %s ] [ t=%s ] %s\n", this->mapLevelToString(level),
           this->tag, humanReadableUpTime, formattedMessageStr);

  esp_log_level_t espLevel = this->mapToEspLogLevel(level);

  esp_log_writev(espLevel, this->tag, finalStrWithPrefix, messageArgs);
}

void UARTLogger::getHumanReadableUpTime(char *timerBuffer) {
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

  uint64_t micros = remainder;

  // Format: HH:MM:SS.mmmmmm e.g. "01:23:45.678901"
  snprintf(timerBuffer, 32, "%02llu:%02llu:%02llu.%06llu", hours, minutes,
           seconds, micros);
}

esp_log_level_t UARTLogger::mapToEspLogLevel(LogLevel level) {
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
const char *UARTLogger::mapLevelToString(LogLevel level) {
  switch (level) {
  case LogLevel::ERROR:
    return "ERROR";
  case LogLevel::WARN:
    return "WARN ";
  case LogLevel::INFO:
    return "INFO ";
  case LogLevel::DEBUG:
    return "DEBUG";
  default:
    return "INFO ";
  }
}

void UARTLogger::error(const char *formattedMessageStr, ...) {
  if (currentLevel < LogLevel::ERROR)
    return;

  va_list messageArgs;
  va_start(messageArgs, formattedMessageStr);
  this->log(LogLevel::ERROR, formattedMessageStr, messageArgs);
  va_end(messageArgs);
}
void UARTLogger::warn(const char *formattedMessageStr, ...) {
  if (currentLevel < LogLevel::WARN)
    return;

  va_list messageArgs;
  va_start(messageArgs, formattedMessageStr);
  this->log(LogLevel::WARN, formattedMessageStr, messageArgs);
  va_end(messageArgs);
}
void UARTLogger::info(const char *formattedMessageStr, ...) {
  if (currentLevel < LogLevel::INFO)
    return;

  va_list messageArgs;
  va_start(messageArgs, formattedMessageStr);
  this->log(LogLevel::INFO, formattedMessageStr, messageArgs);
  va_end(messageArgs);
}
void UARTLogger::debug(const char *formattedMessageStr, ...) {
  if (currentLevel < LogLevel::DEBUG)
    return;

  va_list messageArgs;
  va_start(messageArgs, formattedMessageStr);
  this->log(LogLevel::DEBUG, formattedMessageStr, messageArgs);
  va_end(messageArgs);
}

void UARTLogger::setLevel(LogLevel level) { this->currentLevel = level; }
