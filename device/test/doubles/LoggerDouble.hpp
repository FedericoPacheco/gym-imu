#pragma once

#include <LoggerPort.hpp>

class LoggerDouble : public LoggerPort {
public:
  void setLevel(LogLevel) override {}
  void error(const char *, ...) override {}
  void warn(const char *, ...) override {}
  void info(const char *, ...) override {}
  void debug(const char *, ...) override {}
};
