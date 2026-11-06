#include "ICM42688PSensor.hpp"

ICM42688PSensor::ICM42688PSensor(LoggerPort *logger, gpio_num_t MOSIPin,
                                 gpio_num_t MISOPin, gpio_num_t SCLKPin,
                                 gpio_num_t CSPin)
    : logger(logger), MOSIPin(MOSIPin), MISOPin(MISOPin), SCLKPin(SCLKPin),
      CSPin(CSPin), deviceHandle(nullptr) {}

ICM42688PSensor::~ICM42688PSensor() {
  this->logger->info("Shutting down ICM42688P sensor.");

  if (this->deviceHandle) {
    if (spi_bus_remove_device(this->deviceHandle) != ESP_OK) {
      this->logger->error("Failed to remove sensor from SPI bus.");
      return;
    }
  }

  // Fine since the sensor is the only one using the bus
  if (spi_bus_free(ICM42688PSensor::spiHostId) != ESP_OK) {
    this->logger->error("Failed to free SPI bus.");
    return;
  }

  this->logger->info("ICM42688P sensor shut down successfully.");
}

std::unique_ptr<ICM42688PSensor> ICM42688PSensor::create(LoggerPort *logger,
                                                         gpio_num_t MOSIPin,
                                                         gpio_num_t MISOPin,
                                                         gpio_num_t SCLKPin,
                                                         gpio_num_t CSPin) {

  if (!logger)
    return nullptr;

  std::unique_ptr<ICM42688PSensor> imu(new (std::nothrow) ICM42688PSensor(
      logger, MOSIPin, MISOPin, SCLKPin, CSPin));
  if (!imu) {
    return nullptr;
  }

  if (!imu->initializeSPIBus())
    return nullptr;
  if (!imu->addSensorToSPIBus())
    return nullptr;

  // Temporary diagnostic: if SDO is high-Z, MISO will now read 0xFF instead of
  // 0x00
  // gpio_set_pull_mode(imu->MISOPin, GPIO_PULLUP_ONLY);

  if (!imu->testConnection())
    return nullptr;
  if (!imu->configureSensor())
    return nullptr;
  if (!imu->configurePowerMode())
    return nullptr;

  return imu;
}

bool ICM42688PSensor::initializeSPIBus() {
  this->logger->debug("Initializing SPI bus.");

  spi_bus_config_t busConfig{};
  busConfig.mosi_io_num = this->MOSIPin;
  busConfig.miso_io_num = this->MISOPin;
  busConfig.sclk_io_num = this->SCLKPin;
  busConfig.quadwp_io_num = -1;
  busConfig.quadhd_io_num = -1;
  // data*_io_num allows specifying additional data lines for SPI if needed.
  // This isn't supported by the sensor, so set to -1.
  busConfig.data4_io_num = -1;
  busConfig.data5_io_num = -1;
  busConfig.data6_io_num = -1;
  busConfig.data7_io_num = -1;
  busConfig.flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_SCLK |
                    SPICOMMON_BUSFLAG_MISO | SPICOMMON_BUSFLAG_MOSI;
  const spi_dma_chan_t dmaChannel = SPI_DMA_CH_AUTO;
  esp_err_t result =
      spi_bus_initialize(ICM42688PSensor::spiHostId, &busConfig, dmaChannel);

  if (result != ESP_OK) {
    // Fine since the sensor is the only one using the bus
    spi_bus_free(ICM42688PSensor::spiHostId);
  }
  switch (result) {
  case ESP_OK:
    this->logger->info("SPI bus initialized successfully.");
    return true;
  case ESP_ERR_INVALID_ARG:
    this->logger->error(
        "Invalid argument provided for SPI bus initialization.");
    return false;
  case ESP_ERR_INVALID_STATE:
    this->logger->error("SPI bus is already in use.");
    return false;
  case ESP_ERR_NO_MEM:
    this->logger->error("Not enough memory for SPI bus initialization.");
    return false;
  case ESP_ERR_NOT_FOUND:
    this->logger->error("No available DMA channel.");
    return false;
  default:
    this->logger->error("Unknown error during SPI bus initialization.");
    return false;
  }
}

bool ICM42688PSensor::addSensorToSPIBus() {
  this->logger->debug("Adding sensor to SPI bus.");

  spi_device_interface_config_t deviceConfig{};
  // SPI mode 0: CPOL(clock polarity)=0, CPHA(clock phase)=0.
  // Clock idles low, data sampled on rising edge. // TODO: check this is right
  deviceConfig.mode = 0;
  deviceConfig.clock_speed_hz = 1 * 1000 * 1000; // 1 MHz
  // Chip select pin for the sensor
  deviceConfig.spics_io_num = this->CSPin;
  // From section 9.6: "SPI Interface" of the datasheet, the most significant
  // bit indicates read(1) or write(0) (command), then the remaining 7 bits are
  // the register address.
  deviceConfig.command_bits = 1;
  deviceConfig.address_bits = 7;
  deviceConfig.flags = SPI_DEVICE_HALFDUPLEX;
  // Max ammount of pending SPI transactions
  deviceConfig.queue_size = 1;
  // deviceConfig.pre_cb = nullptr;  // Pre-transfer callback
  // deviceConfig.post_cb = nullptr; // Post-transfer callback
  esp_err_t result = spi_bus_add_device(ICM42688PSensor::spiHostId,
                                        &deviceConfig, &this->deviceHandle);

  if (result != ESP_OK) {
    if (this->deviceHandle)
      spi_bus_remove_device(this->deviceHandle);
  }
  switch (result) {
  case ESP_OK:
    this->logger->info("Sensor added to SPI bus successfully.");
    return true;
  case ESP_ERR_INVALID_ARG:
    this->logger->error("Invalid argument provided for adding sensor to SPI "
                        "bus or configuration combination not supported.");
    return false;
  case ESP_ERR_INVALID_STATE:
    this->logger->error(
        "Selected clock source unavailable or SPI bus not initialized.");
    return false;
  case ESP_ERR_NO_MEM:
    this->logger->error("Not enough memory to add sensor to SPI bus.");
    return false;
  case ESP_ERR_NOT_FOUND:
    this->logger->error("Free CS slots not available.");
    return false;
  default:
    this->logger->error("Unknown error during adding sensor to SPI bus.");
    return false;
  }
}

bool ICM42688PSensor::testConnection() {
  this->logger->debug("Testing SPI connection with the sensor.");

  uint8_t whoAmIBuffer;
  esp_err_t err;
  uint8_t attempts = 0;
  uint8_t MAX_CONNECTION_ATTEMPTS = 10;
  do {
    err = this->spiRead(ICM42688PSensor::Registers::WHO_AM_I, &whoAmIBuffer, 1);
    attempts++;
    if (err == ESP_OK) {
      this->logger->debug("Attempt %d: SPI read result: %02X", attempts,
                          whoAmIBuffer);
    } // if (err != ESP_OK)
    rtosTaskDelay(pdMS_TO_TICKS(2000));
  } while (/* err != ESP_OK && */ attempts < MAX_CONNECTION_ATTEMPTS);

  if (err != ESP_OK) {
    this->logger->error("SPI connection failed after %d attempts", attempts);
    return false;
  }
  // this->logger->debug("SPI connection with the sensor successful. WHO_AM_I "
  //                     "register value: 0x%02X (expected: 0x47)",
  //                     whoAmIBuffer);
  return true;
}

// TODO: implement
bool ICM42688PSensor::configureSensor() { return true; }
// TODO: implement
bool ICM42688PSensor::configurePowerMode() { return true; }
// TODO: implement
std::optional<IMUSample> ICM42688PSensor::readSync() { return std::nullopt; }
// TODO: implement
std::optional<IMUSample> ICM42688PSensor::readAsync() { return std::nullopt; }
// TODO: implement
void ICM42688PSensor::beginAsync() {}
// TODO: implement
void ICM42688PSensor::requestStopAsync() {}

esp_err_t ICM42688PSensor::spiRead(const uint8_t registerAddress,
                                   uint8_t *dataBuffer,
                                   const uint8_t dataLength) {
  if (!dataBuffer || dataLength == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  spi_transaction_t transaction{};
  transaction.cmd = 0x1;
  transaction.addr = registerAddress & ICM42688PSensor::ADDRESS_MASK;
  transaction.length = 0;
  transaction.rxlength = dataLength * 8; // Length in bits
  transaction.rx_buffer = dataBuffer;

  // Send a polling transaction, wait for it to complete, and return the result
  return spi_device_transmit(this->deviceHandle, &transaction);
}