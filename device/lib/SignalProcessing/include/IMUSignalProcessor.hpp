#pragma once
#include <Constants.hpp>
#include <IMUCalibrator.hpp>
#include <IMUEulerGravityRemover.hpp>
#include <IMUEulerOrientationFinder.hpp>
#include <IMUNoiseReducer.hpp>
#include <IMUSensorPort.hpp>
#include <LoggerPort.hpp>
#include <LoopRunner.hpp>
#include <Pipe.hpp>
#include <memory>

// TODO: move to velocity computation strategy interface
typedef LinearAxes VelocitySample; // m/s

class IMUSignalProcessor {
public:
  IMUSignalProcessor(
      std::shared_ptr<Pipe<IMUSample, SAMPLING_PIPE_SIZE>> inputPipe,
      std::shared_ptr<Pipe<IMUSample, TRANSMISSION_PIPE_SIZE>> outputPipe,
      std::unique_ptr<LoopRunner> runner, LoggerPort *logger,
      std::unique_ptr<IMUCalibrator> calibrator,
      std::unique_ptr<IMUNoiseReducer> noiseReducer,
      std::unique_ptr<IMUEulerOrientationFinder> orientationFinder,
      std::unique_ptr<IMUEulerGravityRemover> gravityRemover)
      : inputPipe(inputPipe), outputPipe(outputPipe), runner(std::move(runner)),
        logger(logger), calibrator(std::move(calibrator)),
        noiseReducer(std::move(noiseReducer)),
        orientationFinder(std::move(orientationFinder)),
        gravityRemover(std::move(gravityRemover)){};
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
  std::unique_ptr<IMUEulerOrientationFinder> orientationFinder;
  std::unique_ptr<IMUEulerGravityRemover> gravityRemover;

  static void processingLoopFunction(void *arg) {
    IMUSignalProcessor *self = static_cast<IMUSignalProcessor *>(arg);

    if (!self)
      return;

    std::optional<IMUSample> optSample = self->inputPipe->pop();
    if (optSample.has_value()) {
      IMUSample sample = optSample.value();

      self->calibrator->calibrate(sample);
      self->noiseReducer->filter(sample);
      EulerOrientationSample orientation =
          self->orientationFinder->find(sample);
      self->gravityRemover->remove(sample, orientation);

      self->logger->debug(
          "Processing loop: gravity-free accel = <%.3f, %.3f, %.3f>",
          sample.a.x, sample.a.y, sample.a.z);

      // TODO: temporary until all the components are implemented: must push
      // linear/angular velocities + seqs
      if (!self->outputPipe->push(sample))
        self->logger->warn(
            "Processing loop: failed to push sample to output pipe");
    } else
      self->logger->warn(
          "Processing loop: failed to read sample from input pipe");
  }
};