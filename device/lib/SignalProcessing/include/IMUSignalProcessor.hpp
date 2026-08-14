#pragma once
#include <Constants.hpp>
#include <IMUCalibrator.hpp>
#include <IMUNoiseReducer.hpp>
#include <IMUSensorPort.hpp>
#include <LoggerPort.hpp>
#include <LoopRunner.hpp>
#include <Pipe.hpp>
#include <memory>

typedef AngleAxes EulerOrientationSample; // deg
typedef LinearAxes VelocitySample;        // m/s

class IMUSignalProcessor {
public:
  IMUSignalProcessor(
      std::shared_ptr<Pipe<IMUSample, SAMPLING_PIPE_SIZE>> inputPipe,
      std::shared_ptr<Pipe<IMUSample, TRANSMISSION_PIPE_SIZE>> outputPipe,
      std::unique_ptr<LoopRunner> runner, LoggerPort *logger,
      std::unique_ptr<IMUCalibrator> calibrator,
      std::unique_ptr<IMUNoiseReducer> noiseReducer)
      : inputPipe(inputPipe), outputPipe(outputPipe), runner(std::move(runner)),
        logger(logger), calibrator(std::move(calibrator)),
        noiseReducer(std::move(noiseReducer)){};
  ~IMUSignalProcessor() = default;

  void beginProcessing() {
    if (!this->runner->start(IMUSignalProcessor::processingLoopFunction,
                             static_cast<void *>(this))) {
      logger->error("Failed to start IMU signal processing task");
    }
  }
  void stopProcessing() { this->runner->stop(); }

private:
  std::shared_ptr<Pipe<IMUSample, SAMPLING_PIPE_SIZE>> inputPipe;
  std::shared_ptr<Pipe<IMUSample, TRANSMISSION_PIPE_SIZE>> outputPipe;
  std::unique_ptr<LoopRunner> runner;
  LoggerPort *logger;
  std::unique_ptr<IMUCalibrator> calibrator;
  std::unique_ptr<IMUNoiseReducer> noiseReducer;

  static void processingLoopFunction(void *arg) {
    IMUSignalProcessor *self = static_cast<IMUSignalProcessor *>(arg);

    if (!self)
      return;

    std::optional<IMUSample> optSample = self->inputPipe->pop();
    if (optSample.has_value()) {
      IMUSample sample = optSample.value();

      self->logger->debug(
          "Processing loop: processing sample: a=<%.3f, %.3f, %.3f> m/s2, "
          "w=<%.3f, %.3f, %.3f> deg/s, seq=%u",
          sample.a.x, sample.a.y, sample.a.z, sample.w.roll, sample.w.pitch,
          sample.w.yaw, sample.seq);

      self->calibrator->calibrate(sample);
      self->noiseReducer->filter(sample);

      if (!self->outputPipe->push(sample))
        self->logger->warn(
            "Processing loop: failed to push sample to output pipe");
    } else
      self->logger->warn(
          "Processing loop: failed to read sample from input pipe");
  }
};