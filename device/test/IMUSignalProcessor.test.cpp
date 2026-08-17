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

TEST(IMUSignalProcessor_processingLoopFunction,
     RunsAllOperationsOnASingleSampleSuccessfully) {
  auto inputPipe = std::make_shared<NiceMock<PipeDouble>>();
  auto outputPipe = std::make_shared<NiceMock<PipeDouble>>();
  auto runner = std::make_unique<DeterministicLoopRunner>();
  auto runnerRaw = static_cast<DeterministicLoopRunner *>(runner.get());
  auto calibrator = std::make_unique<NiceMock<IMUCalibratorDouble>>();
  auto noiseReducer = std::make_unique<NiceMock<IMUNoiseReducerDouble>>();
  auto orientationFinder =
      std::make_unique<NiceMock<IMUEulerOrientationFinderDouble>>();
  auto remover = std::make_unique<NiceMock<IMUEulerGravityRemoverDouble>>();
  IMUSample sample = {.a =
                          {
                              .x = 0,
                              .y = 0,
                              .z = 0,
                          },
                      .w =
                          {
                              .roll = 0,
                              .pitch = 0,
                              .yaw = 0,
                          },
                      .seq = 0};

  EXPECT_CALL(*inputPipe, pop(_))
      .WillOnce(Return(std::optional<IMUSample>(sample)));
  EXPECT_CALL(*calibrator, calibrate(_)).Times(1);
  EXPECT_CALL(*noiseReducer, filter(_)).Times(1);
  EXPECT_CALL(*orientationFinder, find(_)).Times(1);
  EXPECT_CALL(*remover, remove(_, _)).Times(1);
  EXPECT_CALL(*outputPipe, push(_)).Times(1).WillOnce(Return(true));

  auto processor =
      IMUSignalProcessor(inputPipe, outputPipe, std::move(runner), &logger,
                         std::move(calibrator), std::move(noiseReducer),
                         std::move(orientationFinder), std::move(remover));
  processor.beginProcessing();
  runnerRaw->runOneStep();
  processor.stopProcessing();
}

TEST(IMUSignalProcessor_processingLoopFunction, DoesNotPushOnNullSamples) {
  auto inputPipe = std::make_shared<NiceMock<PipeDouble>>();
  auto outputPipe = std::make_shared<NiceMock<PipeDouble>>();
  auto runner = std::make_unique<DeterministicLoopRunner>();
  auto runnerRaw = static_cast<DeterministicLoopRunner *>(runner.get());
  auto calibrator = std::make_unique<NiceMock<IMUCalibratorDouble>>();
  auto noiseReducer = std::make_unique<NiceMock<IMUNoiseReducerDouble>>();
  auto orientationFinder =
      std::make_unique<NiceMock<IMUEulerOrientationFinderDouble>>();
  auto remover = std::make_unique<NiceMock<IMUEulerGravityRemoverDouble>>();

  EXPECT_CALL(*inputPipe, pop(_)).WillOnce(Return(std::nullopt));
  EXPECT_CALL(*calibrator, calibrate(_)).Times(0);
  EXPECT_CALL(*noiseReducer, filter(_)).Times(0);
  EXPECT_CALL(*orientationFinder, find(_)).Times(0);
  EXPECT_CALL(*remover, remove(_, _)).Times(0);
  EXPECT_CALL(*outputPipe, push(_)).Times(0);

  auto processor =
      IMUSignalProcessor(inputPipe, outputPipe, std::move(runner), &logger,
                         std::move(calibrator), std::move(noiseReducer),
                         std::move(orientationFinder), std::move(remover));
  processor.beginProcessing();
  runnerRaw->runOneStep();
  processor.stopProcessing();
}

} // namespace