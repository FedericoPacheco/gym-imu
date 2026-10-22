#include "doubles/FreeRTOSDouble.hpp"
#include "doubles/LoggerDouble.hpp"
#include "doubles/PipesDoubles.hpp"
#include <DeterministicLoopRunner.hpp>
#include <IMUGravityRemover.hpp>
#include <IMUOrientationFinder.hpp>
#include <IMUSignalProcessor.hpp>
#include <IMUVelocityEstimator.hpp>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>
#include <optional>

namespace {
using testing::_;
using testing::NiceMock;
using testing::Return;

LoggerDouble logger;
float dt = 1.0f / 30.0f;

class IMUCalibratorDouble : public IMUCalibrator {
public:
  MOCK_METHOD(void, initialize, (IMUSample & sample));
  MOCK_METHOD(void, calibrate, (IMUSample & sample));
  MOCK_METHOD(void, reset, ());
};
class IMUNoiseReducerDouble : public IMUNoiseReducer {
public:
  MOCK_METHOD(void, filter, (IMUSample & sample));
  MOCK_METHOD(void, reset, ());
};
class IMUOrientationFinderDouble : public IMUOrientationFinder<Quaternion> {
public:
  explicit IMUOrientationFinderDouble(float dt)
      : IMUOrientationFinder<Quaternion>(dt) {}
  MOCK_METHOD(void, initialize, (IMUSample & sample));
  MOCK_METHOD(Quaternion, find, (IMUSample & sample));
  MOCK_METHOD(void, reset, ());
};
class IMUGravityRemoverDouble : public IMUGravityRemover<Quaternion> {
public:
  MOCK_METHOD(void, remove, (IMUSample & sample, Quaternion &orientation));
};

class IMUVelocityEstimatorDouble : public IMUVelocityEstimator {
public:
  explicit IMUVelocityEstimatorDouble(float dt) : IMUVelocityEstimator(dt) {}
  MOCK_METHOD(VelocitySample, estimate,
              (IMUSample & sample, Quaternion &orientation));
  MOCK_METHOD(void, reset, ());
};

struct IMUSignalProcessorDependencies {
  std::shared_ptr<NiceMock<PipeDouble>> inputPipe;
  std::shared_ptr<NiceMock<ProcessedPipeDouble>> outputPipe;
  std::unique_ptr<DeterministicLoopRunner> runner;
  std::unique_ptr<NiceMock<IMUCalibratorDouble>> calibrator;
  std::unique_ptr<NiceMock<IMUNoiseReducerDouble>> noiseReducer;
  std::unique_ptr<NiceMock<IMUOrientationFinderDouble>> orientationFinder;
  std::unique_ptr<NiceMock<IMUGravityRemoverDouble>> remover;
  std::unique_ptr<NiceMock<IMUVelocityEstimatorDouble>> velocityEstimator;
  uint32_t initializationSamplesCount;
};
IMUSignalProcessorDependencies buildDefaultDependencies() {
  auto inputPipe = std::make_shared<NiceMock<PipeDouble>>();
  auto outputPipe = std::make_shared<NiceMock<ProcessedPipeDouble>>();
  auto runner = std::make_unique<DeterministicLoopRunner>();
  auto calibrator = std::make_unique<NiceMock<IMUCalibratorDouble>>();
  auto noiseReducer = std::make_unique<NiceMock<IMUNoiseReducerDouble>>();
  auto orientationFinder =
      std::make_unique<NiceMock<IMUOrientationFinderDouble>>(dt);
  auto remover = std::make_unique<NiceMock<IMUGravityRemoverDouble>>();
  auto velocityEstimator =
      std::make_unique<NiceMock<IMUVelocityEstimatorDouble>>(dt);
  uint32_t initializationSamplesCount = 0;

  return {inputPipe,
          outputPipe,
          std::move(runner),
          std::move(calibrator),
          std::move(noiseReducer),
          std::move(orientationFinder),
          std::move(remover),
          std::move(velocityEstimator),
          initializationSamplesCount};
}
int coordinatorTaskToken;

IMUSignalProcessor *getInstanceWith(IMUSignalProcessorDependencies &&deps) {
  IMUSignalProcessor::resetInstanceForTests();
  resetFreeRTOSPortFakes();
  rtosCreateMutexStatic_fake.return_val =
      reinterpret_cast<SemaphoreHandle_t>(0x1);
  rtosSemaphoreTake_fake.return_val = pdTRUE;
  rtosSemaphoreGive_fake.return_val = pdTRUE;
  return IMUSignalProcessor::getInstance(
      deps.inputPipe, deps.outputPipe, std::move(deps.runner), &logger,
      std::move(deps.calibrator), std::move(deps.noiseReducer),
      std::move(deps.orientationFinder), std::move(deps.remover),
      std::move(deps.velocityEstimator), deps.initializationSamplesCount,
      &coordinatorTaskToken);
}

TEST(IMUSignalProcessor_processingLoopFunction,
     InitializesPipelineSuccessfullyBeforeProcessing) {
  auto deps = buildDefaultDependencies();
  deps.initializationSamplesCount = 1;
  auto runnerRaw = static_cast<DeterministicLoopRunner *>(deps.runner.get());

  EXPECT_CALL(*deps.inputPipe, pop(_))
      .WillOnce(Return(std::optional<IMUSample>(IMUSample{})));
  EXPECT_CALL(*deps.calibrator, initialize(_)).Times(1);
  EXPECT_CALL(*deps.orientationFinder, initialize(_)).Times(1);
  EXPECT_CALL(*deps.outputPipe, push(_)).Times(0);

  auto *processor = getInstanceWith(std::move(deps));
  processor->beginProcessing();
  runnerRaw->runOneStep();
  processor->requestStopProcessing();

  IMUSignalProcessor::resetInstanceForTests();
}

TEST(IMUSignalProcessor_processingLoopFunction,
     RunsAllOperationsOnASingleSampleSuccessfullyAfterInitialization) {
  auto deps = buildDefaultDependencies();
  auto runnerRaw = static_cast<DeterministicLoopRunner *>(deps.runner.get());

  EXPECT_CALL(*deps.inputPipe, pop(_))
      .WillOnce(Return(std::optional<IMUSample>(IMUSample{})));
  EXPECT_CALL(*deps.calibrator, calibrate(_)).Times(1);
  EXPECT_CALL(*deps.noiseReducer, filter(_)).Times(1);
  EXPECT_CALL(*deps.orientationFinder, find(_)).Times(1);
  EXPECT_CALL(*deps.remover, remove(_, _)).Times(1);
  EXPECT_CALL(*deps.velocityEstimator, estimate(_, _)).Times(1);
  EXPECT_CALL(*deps.outputPipe, push(_)).Times(1).WillOnce(Return(true));

  auto *processor = getInstanceWith(std::move(deps));
  processor->beginProcessing();
  runnerRaw->runOneStep();
  processor->requestStopProcessing();

  IMUSignalProcessor::resetInstanceForTests();
}

TEST(IMUSignalProcessor_processingLoopFunction,
     DoesNotPushOnNullSamplesAfterInitialization) {
  auto deps = buildDefaultDependencies();
  auto runnerRaw = static_cast<DeterministicLoopRunner *>(deps.runner.get());

  EXPECT_CALL(*deps.inputPipe, pop(_)).WillOnce(Return(std::nullopt));
  EXPECT_CALL(*deps.calibrator, calibrate(_)).Times(0);
  EXPECT_CALL(*deps.noiseReducer, filter(_)).Times(0);
  EXPECT_CALL(*deps.orientationFinder, find(_)).Times(0);
  EXPECT_CALL(*deps.remover, remove(_, _)).Times(0);
  EXPECT_CALL(*deps.velocityEstimator, estimate(_, _)).Times(0);
  EXPECT_CALL(*deps.outputPipe, push(_)).Times(0);

  auto *processor = getInstanceWith(std::move(deps));
  processor->beginProcessing();
  runnerRaw->runOneStep();
  processor->requestStopProcessing();

  IMUSignalProcessor::resetInstanceForTests();
}

TEST(IMUSignalProcessor_processingLoopFunction, ResetsAfterStoppingProcessing) {
  auto deps = buildDefaultDependencies();
  auto runnerRaw = static_cast<DeterministicLoopRunner *>(deps.runner.get());

  EXPECT_CALL(*deps.inputPipe, pop(false)).WillOnce(Return(std::nullopt));
  EXPECT_CALL(*deps.calibrator, reset()).Times(1);
  EXPECT_CALL(*deps.noiseReducer, reset()).Times(1);
  EXPECT_CALL(*deps.orientationFinder, reset()).Times(1);
  EXPECT_CALL(*deps.velocityEstimator, reset()).Times(1);

  auto *processor = getInstanceWith(std::move(deps));
  processor->beginProcessing();
  processor->requestStopProcessing();
  runnerRaw->runOneStep();

  IMUSignalProcessor::resetInstanceForTests();
}

TEST(IMUSignalProcessor_processingLoopFunction,
     DoesntResetIfThereAreRemainingSamples) {
  auto deps = buildDefaultDependencies();
  auto runnerRaw = static_cast<DeterministicLoopRunner *>(deps.runner.get());

  EXPECT_CALL(*deps.inputPipe, pop(false))
      .WillOnce(Return(std::optional<IMUSample>(IMUSample{})));
  EXPECT_CALL(*deps.calibrator, reset()).Times(0);
  EXPECT_CALL(*deps.noiseReducer, reset()).Times(0);
  EXPECT_CALL(*deps.orientationFinder, reset()).Times(0);
  EXPECT_CALL(*deps.velocityEstimator, reset()).Times(0);

  auto *processor = getInstanceWith(std::move(deps));
  processor->beginProcessing();
  processor->requestStopProcessing();
  runnerRaw->runOneStep();

  IMUSignalProcessor::resetInstanceForTests();
}

} // namespace