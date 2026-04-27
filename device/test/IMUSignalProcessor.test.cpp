#include "doubles/LoggerDouble.hpp"
#include "doubles/PipeDouble.hpp"
#include <DeterministicLoopRunner.hpp>
#include <IMUSignalProcessor.hpp>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
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

TEST(IMUSignalProcessor_processingLoopFunction,
     RunsAllOperationsOnASingleSampleSuccessfully) {
  auto inputPipe = std::make_shared<NiceMock<PipeDouble>>();
  auto outputPipe = std::make_shared<NiceMock<PipeDouble>>();
  auto runner = std::make_unique<DeterministicLoopRunner>();
  auto runnerRaw = static_cast<DeterministicLoopRunner *>(runner.get());
  auto calibrator = std::make_unique<NiceMock<IMUCalibratorDouble>>();

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
                      .t = 0};

  EXPECT_CALL(*inputPipe, pop(_))
      .WillOnce(Return(std::optional<IMUSample>(sample)));
  EXPECT_CALL(*calibrator, calibrate(_)).Times(1);
  EXPECT_CALL(*outputPipe, push(_)).Times(1).WillOnce(Return(true));

  auto processor = IMUSignalProcessor(inputPipe, outputPipe, std::move(runner),
                                      &logger, std::move(calibrator));
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

  EXPECT_CALL(*inputPipe, pop(_)).WillOnce(Return(std::nullopt));
  EXPECT_CALL(*calibrator, calibrate(_)).Times(0);
  EXPECT_CALL(*outputPipe, push(_)).Times(0);

  auto processor = IMUSignalProcessor(inputPipe, outputPipe, std::move(runner),
                                      &logger, std::move(calibrator));
  processor.beginProcessing();
  runnerRaw->runOneStep();
  processor.stopProcessing();
}

} // namespace