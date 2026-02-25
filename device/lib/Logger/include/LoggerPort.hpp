#pragma once

enum class LogLevel { ERROR, WARN, INFO, DEBUG };

class LoggerPort {
public:
  virtual ~LoggerPort() = default;

  virtual void setLevel(LogLevel level) = 0;

  virtual void error(const char *format, ...) = 0;
  virtual void warn(const char *format, ...) = 0;
  virtual void info(const char *format, ...) = 0;
  virtual void debug(const char *format, ...) = 0;
};
