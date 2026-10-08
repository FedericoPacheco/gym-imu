#include "IMUEulerOrientationFinder.hpp"
#include "doubles/LoggerDouble.hpp"
#include "doubles/PipeDouble.hpp"
#include <DeterministicLoopRunner.hpp>
#include <IMUEulerGravityRemover.hpp>
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

class IMUCalibratorDouble : public IMUCalibrator {
public:
  MOCK_METHOD(void, calibrate, (IMUSample & sample));
};
class IMUNoiseReducerDouble : public IMUNoiseReducer {
public:
  MOCK_METHOD(void, filter, (IMUSample & sample));
};
class IMUEulerOrientationFinderDouble : public IMUEulerOrientationFinder {
public:
  MOCK_METHOD(EulerOrientationSample, find, (IMUSample & sample));
};
class IMUEulerGravityRemoverDouble : public IMUEulerGravityRemover {
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
  std::unique_ptr<NiceMock<IMUEulerOrientationFinderDouble>> orientationFinder;
  std::unique_ptr<NiceMock<IMUEulerGravityRemoverDouble>> remover;
};
IMUSignalProcessorDependencies buildDefaultDependencies() {
  auto inputPipe = std::make_shared<NiceMock<PipeDouble>>();
  auto outputPipe = std::make_shared<NiceMock<PipeDouble>>();
  auto runner = std::make_unique<DeterministicLoopRunner>();
  auto calibrator = std::make_unique<NiceMock<IMUCalibratorDouble>>();
  auto noiseReducer = std::make_unique<NiceMock<IMUNoiseReducerDouble>>();
  auto orientationFinder =
      std::make_unique<NiceMock<IMUEulerOrientationFinderDouble>>();
  auto remover = std::make_unique<NiceMock<IMUEulerGravityRemoverDouble>>();

  return {inputPipe,
          outputPipe,
          std::move(runner),
          std::move(calibrator),
          std::move(noiseReducer),
          std::move(orientationFinder),
          std::move(remover)};
}
IMUSignalProcessor *getInstanceWith(IMUSignalProcessorDependencies &&deps) {
  IMUSignalProcessor::resetInstanceForTests();
  return IMUSignalProcessor::getInstance(
      deps.inputPipe, deps.outputPipe, std::move(deps.runner), &logger,
      std::move(deps.calibrator), std::move(deps.noiseReducer),
      std::move(deps.orientationFinder), std::move(deps.remover));
}

TEST(IMUSignalProcessor_processingLoopFunction,
     RunsAllOperationsOnASingleSampleSuccessfully) {
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

TEST(IMUSignalProcessor_processingLoopFunction, DoesNotPushOnNullSamples) {
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

} // namespace