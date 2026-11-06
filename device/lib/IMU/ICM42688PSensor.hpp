#pragma once
#include "driver/spi_master.h"
#include <IMUSensorPort.hpp>
#include <LoggerPort.hpp>
#include <memory>
#include <optional>
#include <ports/ESP-IDF/ESPIDFPort.hpp>
#include <ports/FreeRTOS/FreeRTOSPort.hpp>

/** Left unfinished for now until I receive the correct sensor. After days of
debugging I discovered I received a BMI323 instead of an ICM42688P, which has a
different behavior and register map and lower performance. Figuring this out
was the purpose of the temporary ICM42688PI2CProbe and BMI323I2CProbe classes,
which will be commited for now.
*/

/**
Wiring:
Sensor: Logic analyzer
GND: GND
SCL: CH1
SDA: CH2
SAO: CH3
CS: CH4
INT1: CH5

Software configuration (PulseView):
Add protocol decoder > Search "SPI" > Set the following parameters:
CLK: D0,
MISO: D2,
MOSI: D1,
CS: D3,
CS polarity: active-low
Clock polarity: 0,
Clock phase: 0,
Bit order: msb first
Make sure that: analyzer sampling frequency >>> SPI clock frequency
*/

class ICM42688PSensor : public IMUSensorPort {
public:
  static std::unique_ptr<ICM42688PSensor>
  create(LoggerPort *logger, gpio_num_t MOSIPin, gpio_num_t MISOPin,
         gpio_num_t SCLKPin, gpio_num_t CSPin);
  ~ICM42688PSensor() override;

  std::optional<IMUSample> readSync() override;
  std::optional<IMUSample> readAsync() override;
  void beginAsync() override;
  void requestStopAsync() override;

private:
  static constexpr spi_host_device_t spiHostId = SPI2_HOST;
  static constexpr uint8_t ADDRESS_MASK = 0x7F;

  enum Registers : uint8_t {
    WHO_AM_I = 0x75,
    // REG_BANK_SEL = 0x76,
    GYRO_CONFIG0 = 0x4f,
    ACCEL_CONFIG0 = 0X50
  };

  ICM42688PSensor(LoggerPort *logger, gpio_num_t MOSIPin, gpio_num_t MISOPin,
                  gpio_num_t SCLKPin, gpio_num_t CSPin);

  LoggerPort *logger;
  gpio_num_t MOSIPin, MISOPin, SCLKPin, CSPin;
  spi_device_handle_t deviceHandle;

  bool initializeSPIBus();
  bool addSensorToSPIBus();
  bool testConnection();
  bool configureSensor();
  bool configurePowerMode();

  esp_err_t spiRead(const uint8_t registerAddress, uint8_t *dataBuffer,
                    const uint8_t dataLength);
};