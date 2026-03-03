#pragma once

#include <Constants.hpp>
#include <IMUSensorPort.hpp>
#include <Pipe.hpp>
#include <gmock/gmock.h>

class PipeDouble : public Pipe<IMUSample, SAMPLING_PIPE_SIZE> {
public:
  MOCK_METHOD(bool, push, (const IMUSample &item), (override));
  MOCK_METHOD(std::optional<IMUSample>, pop, (bool blockWhenEmpty), (override));
  MOCK_METHOD(uint32_t, itemsFilled, (), (const, override));
  MOCK_METHOD(uint32_t, itemsLeft, (), (const, override));
  MOCK_METHOD(PipeMetrics, getMetrics, (), (const, override));
};
