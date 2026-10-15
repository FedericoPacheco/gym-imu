#include "IMUOrientationFinder.hpp"
#include "doubles/LoggerDouble.hpp"
#include "doubles/PipeDouble.hpp"
#include <DeterministicLoopRunner.hpp>
#include <IMUGravityRemover.hpp>
#include <IMUSignalProcessor.hpp>
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
};
class IMUOrientationFinderDouble
    : public IMUOrientationFinder<EulerOrientationSample> {
public:
  explicit IMUOrientationFinderDouble(float dt)
      : IMUOrientationFinder<EulerOrientationSample>(dt) {}
  MOCK_METHOD(void, initialize, (IMUSample & sample));
  MOCK_METHOD(EulerOrientationSample, find, (IMUSample & sample));
  MOCK_METHOD(void, reset, ());
};
class IMUGravityRemoverDouble
    : public IMUGravityRemover<EulerOrientationSample> {
public:
  MOCK_METHOD(void, remove,
              (IMUSample & sample, EulerOrientationSample &orientation));
};

struct IMUSignalProcessorDependencies {
  std::shared_ptr<NiceMock<PipeDouble>> inputPipe;
  std::shared_ptr<NiceMock<PipeDouble>> outputPipe;
  std::unique_ptr<DeterministicLoopRunner> runner;
  std::unique_ptr<NiceMock<IMUCalibratorDouble>> calibrator;
  std::unique_ptr<NiceMock<IMUNoiseReducerDouble>> noiseReducer;
  std::unique_ptr<NiceMock<IMUOrientationFinderDouble>> orientationFinder;
  std::unique_ptr<NiceMock<IMUGravityRemoverDouble>> remover;
  uint32_t initializationSamplesCount;
};
IMUSignalProcessorDependencies buildDefaultDependencies() {
  auto inputPipe = std::make_shared<NiceMock<PipeDouble>>();
  auto outputPipe = std::make_shared<NiceMock<PipeDouble>>();
  auto runner = std::make_unique<DeterministicLoopRunner>();
  auto calibrator = std::make_unique<NiceMock<IMUCalibratorDouble>>();
  auto noiseReducer = std::make_unique<NiceMock<IMUNoiseReducerDouble>>();
  auto orientationFinder =
      std::make_unique<NiceMock<IMUOrientationFinderDouble>>(dt);
  auto remover = std::make_unique<NiceMock<IMUGravityRemoverDouble>>();
  uint32_t initializationSamplesCount = 0;

  return {inputPipe,
          outputPipe,
          std::move(runner),
          std::move(calibrator),
          std::move(noiseReducer),
          std::move(orientationFinder),
          std::move(remover),
          initializationSamplesCount};
}
IMUSignalProcessor *getInstanceWith(IMUSignalProcessorDependencies &&deps) {
  IMUSignalProcessor::resetInstanceForTests();
  return IMUSignalProcessor::getInstance(
      deps.inputPipe, deps.outputPipe, std::move(deps.runner), &logger,
      std::move(deps.calibrator), std::move(deps.noiseReducer),
      std::move(deps.orientationFinder), std::move(deps.remover),
      deps.initializationSamplesCount);
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
  processor->stopProcessing();

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
  EXPECT_CALL(*deps.outputPipe, push(_)).Times(1).WillOnce(Return(true));

  auto *processor = getInstanceWith(std::move(deps));
  processor->beginProcessing();
  runnerRaw->runOneStep();
  processor->stopProcessing();

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
  EXPECT_CALL(*deps.outputPipe, push(_)).Times(0);

  auto *processor = getInstanceWith(std::move(deps));
  processor->beginProcessing();
  runnerRaw->runOneStep();
  processor->stopProcessing();

  IMUSignalProcessor::resetInstanceForTests();
}

TEST(IMUSignalProcessor_processingLoopFunction, ResetsAfterStoppingProcessing) {
  auto deps = buildDefaultDependencies();
  auto runnerRaw = static_cast<DeterministicLoopRunner *>(deps.runner.get());

  EXPECT_CALL(*deps.inputPipe, itemsLeft()).Times(1).WillOnce(Return(0));
  EXPECT_CALL(*deps.calibrator, reset()).Times(1);
  EXPECT_CALL(*deps.orientationFinder, reset()).Times(1);

  auto *processor = getInstanceWith(std::move(deps));
  processor->beginProcessing();
  processor->stopProcessing();
  runnerRaw->runOneStep();

  IMUSignalProcessor::resetInstanceForTests();
}

TEST(IMUSignalProcessor_processingLoopFunction,
     DoesntResetIfThereAreRemainingSamples) {
  auto deps = buildDefaultDependencies();
  auto runnerRaw = static_cast<DeterministicLoopRunner *>(deps.runner.get());

  EXPECT_CALL(*deps.inputPipe, itemsLeft()).Times(1).WillOnce(Return(1));
  EXPECT_CALL(*deps.calibrator, reset()).Times(0);
  EXPECT_CALL(*deps.orientationFinder, reset()).Times(0);

  auto *processor = getInstanceWith(std::move(deps));
  processor->beginProcessing();
  processor->stopProcessing();
  runnerRaw->runOneStep();

  IMUSignalProcessor::resetInstanceForTests();
}

} // namespace