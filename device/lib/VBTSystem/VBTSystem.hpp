#pragma once

#include <Button.hpp>
#include <Constants.hpp>
#include <IMUSensorPort.hpp>
#include <IMUSignalProcessor.hpp>
#include <LED.hpp>
#include <LoggerPort.hpp>
#include <Pipe.hpp>
#include <memory>
#include <new>
#include <ports/BLE/BLETransmitterPort.hpp>
#include <ports/ESP-IDF/ESPIDFPort.hpp>
#include <ports/FreeRTOS/FreeRTOSPort.hpp>

class VBTSystem {
public:
  struct Dependencies {
    IMUSensorPort *imu;
    BLETransmitterPort *ble;
    IMUSignalProcessor *processor;
    Button *button;
    LED *led;
    PipeMetricsSource *samplingPipe;
    PipeMetricsSource *transmissionPipe;
    LoggerPort *logger;
    LoggerPort *samplingPipeLogger;
    LoggerPort *transmissionPipeLogger;
    bool signalProcessingEnabled;
  };

  static VBTSystem *getInstance(const Dependencies &dependencies);

  bool run();
  bool stopPipeline();

private:
  explicit VBTSystem(const Dependencies &dependencies);

  IMUSensorPort *imu;
  BLETransmitterPort *ble;
  IMUSignalProcessor *processor;
  Button *button;
  LED *led;
  PipeMetricsSource *samplingPipe;
  PipeMetricsSource *transmissionPipe;
  LoggerPort *logger;
  LoggerPort *samplingPipeLogger;
  LoggerPort *transmissionPipeLogger;
  bool signalProcessingEnabled;

  void startPipeline();
  bool waitForStop(uint32_t notificationMask, const char *stage);
  void updateLed(bool isRecording);
  void logPipeMetrics();
};
