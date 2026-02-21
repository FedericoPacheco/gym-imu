#include <MPU6050Sensor.hpp>

std::unique_ptr<MPU6050Sensor> MPU6050Sensor::instance = nullptr;

MPU6050Sensor::MPU6050Sensor(
    Logger *logger, std::shared_ptr<Pipe<IMUSample, SAMPLING_PIPE_SIZE>> pipe,
    gpio_num_t INTPin, gpio_num_t SDAPin, gpio_num_t SCLPin,
    int samplingFrequencyHz)
    : logger(logger), pipe(pipe), bus(I2C_NUM_0) {

  this->logger->debug("Sensor parameters: INTPin=%d, SDAPin=%d, SCLPin=%d, "
                      "samplingFrequencyHz=%d",
                      INTPin, SDAPin, SCLPin, samplingFrequencyHz);

  this->INTPin = INTPin;
  this->SDAPin = SDAPin;
  this->SCLPin = SCLPin;
  this->samplingFrequencyHz = samplingFrequencyHz;
}

std::unique_ptr<MPU6050Sensor>
MPU6050Sensor::create(Logger *logger,
                      std::shared_ptr<Pipe<IMUSample, SAMPLING_PIPE_SIZE>> pipe,
                      gpio_num_t INTPin, gpio_num_t SDAPin, gpio_num_t SCLPin,
                      int samplingFrequencyHz) {

  // nothrow: in case of failure, return nullptr instead of throwing
  std::unique_ptr<MPU6050Sensor> imu(new (std::nothrow) MPU6050Sensor(
      logger, pipe, INTPin, SDAPin, SCLPin, samplingFrequencyHz));

  if (!imu) {
    if (logger)
      logger->error("Failed to allocate MPU6050Sensor");
    return nullptr;
  }

  if (!pipe) {
    if (logger)
      logger->error("Pipe pointer is null");
    return nullptr;
  }

  if (!imu->initializeI2CBus())
    return nullptr;
  if (!imu->resetSensor())
    return nullptr;
  // imu->performDiagnostics();
  if (!imu->testConnection())
    return nullptr;
  if (!imu->initializeSensor())
    return nullptr;
  if (!imu->configureSettings())
    return nullptr;
  if (!imu->setupDMPQueue())
    return nullptr;
  // Leave everything ready BEFORE enabling interrupts
  if (!imu->setupTask())
    return nullptr;
  if (!imu->configureInterrupts())
    return nullptr;

  logger->info("MPU6050 sensor initialized successfully");

  return imu;
}

MPU6050Sensor *MPU6050Sensor::getInstance(
    Logger *logger, std::shared_ptr<Pipe<IMUSample, SAMPLING_PIPE_SIZE>> pipe,
    gpio_num_t INTPin, gpio_num_t SDAPin, gpio_num_t SCLPin,
    int samplingFrequencyHz) {

  if (!MPU6050Sensor::instance)
    MPU6050Sensor::instance = MPU6050Sensor::create(
        logger, pipe, INTPin, SDAPin, SCLPin, samplingFrequencyHz);
  return MPU6050Sensor::instance.get();
}

bool MPU6050Sensor::initializeI2CBus() {
  this->logger->debug("Initializing I2C bus");

  RETURN_FALSE_ON_ERROR(
      this->bus.begin(this->SDAPin, this->SCLPin, GPIO_PULLUP_DISABLE,
                      GPIO_PULLUP_DISABLE, MPU6050Sensor::BUS_FREQUENCY_HZ),
      this->logger, "I2C bus initialization failed");

  this->sensor.setBus(this->bus);
  RETURN_FALSE_ON_ERROR(this->sensor.lastError(), this->logger,
                        "Failed to set I2C bus for sensor");

  this->sensor.setAddr(mpud::MPU_I2CADDRESS_AD0_LOW);
  RETURN_FALSE_ON_ERROR(this->sensor.lastError(), this->logger,
                        "Failed to set I2C address for sensor");

  return true;
}

bool MPU6050Sensor::resetSensor() {
  this->logger->debug("Performing sensor reset");

  RETURN_FALSE_ON_ERROR(this->sensor.reset(), this->logger, "Reset failed");
  vTaskDelay(pdMS_TO_TICKS(250));

  return true;
}

void MPU6050Sensor::performDiagnostics() {
  this->logger->debug("Scanning I2C bus for MPU6050 device");
  this->bus.scanner();

  uint8_t wai = this->sensor.whoAmI();
  this->logger->debug("Retrieved sensor's WHO_AM_I register: 0x%02X", wai);

  this->logger->debug("Retrieved sensor's register dump (0x00..0x1F):");
  this->sensor.registerDump(0x00, 0x1F);
  mpud::selftest_t selfTestResult = 0;
  esp_err_t serr = this->sensor.selfTest(&selfTestResult);
  this->logger->info("selfTest result: 0x%X err=%s", selfTestResult,
                     esp_err_to_name(serr));
}

bool MPU6050Sensor::testConnection() {
  this->logger->debug("Attempting I2C connection");

  uint8_t attempts = 0;
  uint8_t MAX_CONNECTION_ATTEMPTS = 5;
  esp_err_t err = this->sensor.testConnection();
  while (err != ESP_OK && attempts < MAX_CONNECTION_ATTEMPTS) {
    this->logger->error("I2C connection failed: %s (%#X)", esp_err_to_name(err),
                        err);
    vTaskDelay(pdMS_TO_TICKS(1000));
    err = this->sensor.testConnection();
    attempts++;
  }
  if (attempts == MAX_CONNECTION_ATTEMPTS) {
    this->logger->error("I2C connection failed after %d attempts", attempts);
    return false;
  }

  this->logger->debug("I2C connection successful");
  return true;
}

bool MPU6050Sensor::initializeSensor() {
  this->logger->debug("Initializing MPU6050 sensor");

  RETURN_FALSE_ON_ERROR(this->sensor.initialize(), this->logger,
                        "MPU initialization failed");

  return true;
}

bool MPU6050Sensor::configureSettings() {
  this->logger->debug("Configuring sensor settings");

  RETURN_FALSE_ON_ERROR(this->sensor.setSampleRate(this->samplingFrequencyHz),
                        this->logger, "Failed to set sample rate");

  RETURN_FALSE_ON_ERROR(
      this->sensor.setAccelFullScale(MPU6050Sensor::ACCELEROMETER_SCALE),
      this->logger, "Failed to set accelerometer scale");

  RETURN_FALSE_ON_ERROR(
      this->sensor.setGyroFullScale(MPU6050Sensor::GYROSCOPE_SCALE),
      this->logger, "Failed to set gyroscope scale");

  return true;
}

bool MPU6050Sensor::setupDMPQueue() {
  this->logger->debug("Setting up DMP FIFO queue on sensor");

  RETURN_FALSE_ON_ERROR(
      this->sensor.setFIFOConfig(mpud::FIFO_CFG_ACCEL | mpud::FIFO_CFG_GYRO),
      this->logger, "Failed to configure FIFO");

  RETURN_FALSE_ON_ERROR(this->sensor.setFIFOEnabled(true), this->logger,
                        "Failed to enable FIFO");

  return true;
}

bool MPU6050Sensor::setupTask() {
  this->logger->debug("Setting up FreeRTOS task for async reading");

  this->setDoRead(false);
  BaseType_t taskResult = xTaskCreate(
      readTask, "readTask", MPU6050Sensor::READ_TASK_STACK_SIZE, this,
      MPU6050Sensor::READ_TASK_PRIORITY, &this->readTaskHandle);
  if (taskResult != pdPASS) {
    this->logger->error("Failed to create read task");
    return false;
  }

  return true;
}

bool MPU6050Sensor::configureInterrupts() {
  this->logger->debug("Configuring pin and interrupts");
  const gpio_config_t pinConfig{.pin_bit_mask = (uint64_t)0x1 << this->INTPin,
                                .mode = GPIO_MODE_INPUT,
                                .pull_up_en = GPIO_PULLUP_DISABLE,
                                .pull_down_en = GPIO_PULLDOWN_ENABLE,
                                .intr_type = GPIO_INTR_POSEDGE};
  RETURN_FALSE_ON_ERROR(gpio_config(&pinConfig), this->logger,
                        "GPIO config failed");

  RETURN_FALSE_ON_ERROR(
      gpio_isr_handler_add(this->INTPin, MPU6050Sensor::isrHandler, this),
      this->logger, "ISR handler add failed");
  const mpud::int_config_t intConfig{.level = mpud::INT_LVL_ACTIVE_HIGH,
                                     .drive = mpud::INT_DRV_PUSHPULL,
                                     .mode = mpud::INT_MODE_PULSE50US,
                                     .clear = mpud::INT_CLEAR_STATUS_REG};
  // gpio_install_isr_service() assumed to be called from app_main()

  RETURN_FALSE_ON_ERROR(this->sensor.setInterruptConfig(intConfig),
                        this->logger, "Interrupt config failed");

  RETURN_FALSE_ON_ERROR(
      this->sensor.setInterruptEnabled(mpud::INT_EN_RAWDATA_READY),
      this->logger, "Interrupt enable failed");
  return true;
}

MPU6050Sensor::~MPU6050Sensor() {
  this->logger->info("Shutting down MPU6050 sensor");

  this->setDoRead(false);

  this->logger->debug("Notifying read task to unblock and finish cleanly");
  if (this->readTaskHandle) {
    xTaskNotifyGive(this->readTaskHandle);
    vTaskDelay(1);
    vTaskDelete(this->readTaskHandle);
  }

  this->logger->debug("Removing ISR handler");
  gpio_isr_handler_remove(this->INTPin);

  this->logger->info("MPU6050 sensor shut down successfully");
}

inline void MPU6050Sensor::setDoRead(bool value) {
  this->doRead.store(value, std::memory_order_relaxed);
}
inline bool MPU6050Sensor::getDoRead() {
  return this->doRead.load(std::memory_order_relaxed);
}

std::optional<IMUSample> MPU6050Sensor::readAsync() {
  std::optional<IMUSample> sample = this->pipe->pop();
  if (sample) {
    this->logger->info("Async read: a=<%.3f, %.3f, %.3f> m/s2, "
                       "w=<%.3f, %.3f, %.3f> deg/s, t=%lld us",
                       sample->a.x, sample->a.y, sample->a.z, sample->w.roll,
                       sample->w.pitch, sample->w.yaw, sample->t);
    return sample;
  }
  this->logger->warn("Async read: failed");
  return std::nullopt;
}

void IRAM_ATTR MPU6050Sensor::isrHandler(void *arg) {
  MPU6050Sensor *self = static_cast<MPU6050Sensor *>(arg);

  if (!self || !self->readTaskHandle)
    return;

  // Notify the read task to process the interrupt
  BaseType_t highPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(self->readTaskHandle, &highPriorityTaskWoken);
  if (highPriorityTaskWoken == pdTRUE)
    portYIELD_FROM_ISR(); // Context switch if needed
}

void MPU6050Sensor::readTask(void *arg) {
  // Get the instance pointer
  MPU6050Sensor *self = static_cast<MPU6050Sensor *>(arg);

  self->logger->debug("Read task started");

  while (true) {
    // Block and wait for ISR notification
    uint32_t notificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (!self->getDoRead()) {
      continue;
    }

    if (notificationValue > 1) {
      self->logger->warn(
          "Read task: multiple notifications received (%d), processing backlog",
          notificationValue);
    }

    bool wasCountReadError = self->sensor.lastError() != ESP_OK;
    if (wasCountReadError) {
      self->logger->warn("Read task: FIFO count read error");
      self->sensor.resetFIFO();
      continue;
    }

    uint16_t initialFifoCount = self->sensor.getFIFOCount();
    bool isFifoMisaligned =
        initialFifoCount % MPU6050Sensor::FIFO_PACKET_SIZE != 0;
    if (isFifoMisaligned) {
      self->logger->warn("Read task: FIFO misaligned (count=%d)",
                         initialFifoCount);
      self->sensor.resetFIFO();
      continue;
    }

    self->batchReadDMPQueue(initialFifoCount);
  }
}

void MPU6050Sensor::batchReadDMPQueue(uint16_t initialFifoCount) {
  bool areManyPacketsAvailable =
      initialFifoCount > 2 * MPU6050Sensor::FIFO_PACKET_SIZE;
  if (areManyPacketsAvailable)
    this->logger->warn("Read task: many packets available (count=%d), "
                       "attempting to process backlog",
                       initialFifoCount);

  int packetsProcessed = 0;
  bool wasBufferReadError = false;
  int currentFifoCount = (int)initialFifoCount;
  uint8_t sensorBuffer[MPU6050Sensor::FIFO_PACKET_SIZE];
  while (currentFifoCount >= MPU6050Sensor::FIFO_PACKET_SIZE &&
         packetsProcessed < MPU6050Sensor::READ_TASK_MAX_BATCH) {

    wasBufferReadError = this->sensor.readFIFO(MPU6050Sensor::FIFO_PACKET_SIZE,
                                               sensorBuffer) != ESP_OK;
    if (wasBufferReadError) {
      this->logger->warn(
          "Read task: FIFO buffer read error during batch processing");
      this->sensor.resetFIFO();
      break; // Exit loop on error
    }

    auto [aRaw, wRaw] = this->parseSensorData(sensorBuffer);
    IMUSample sample = this->toIMUSample(aRaw, wRaw);
    this->logger->debug("Read task: processing packet %d: a=<%.3f, %.3f, "
                        "%.3f> m/s2, w=<%.3f, %.3f, %.3f> deg/s, t=%lld us",
                        packetsProcessed + 1, sample.a.x, sample.a.y,
                        sample.a.z, sample.w.roll, sample.w.pitch, sample.w.yaw,
                        sample.t);

    if (!this->pipe->push(sample)) {
      this->logger->warn("Read task: failed to push sample");
      // Sample is dropped, but continue processing FIFO
    }

    currentFifoCount -= MPU6050Sensor::FIFO_PACKET_SIZE;
    packetsProcessed++;
  }

  // Reset as fallback to avoid overflow
  areManyPacketsAvailable =
      currentFifoCount > 2 * MPU6050Sensor::FIFO_PACKET_SIZE;
  if (areManyPacketsAvailable) {
    this->logger->warn("Read task: FIFO still overloaded after processing "
                       "(count=%d), resetting",
                       currentFifoCount);
    this->sensor.resetFIFO();
  }
}

std::tuple<mpud::raw_axes_t, mpud::raw_axes_t>
MPU6050Sensor::parseSensorData(const uint8_t *data) {
  mpud::raw_axes_t aRaw{}, wRaw{};

  // Explanation:
  // Sensor's buffer stores data in big-endian format (i.e. most significant
  // byte first). If it's type is "uint8_t", it means each index is one byte.
  // Since each axis is 2 bytes long, we need to move the first byte left by
  // 1 byte = 8 bits with the bitwise shift left operator <<, and then
  // concatenate it with the second byte using the bitwise OR operator (|).
  // Recall that when shifting left a trail of zeros is left on the right,
  // and 0 OR x = x.
  aRaw.x = data[0] << 8 | data[1];
  aRaw.y = data[2] << 8 | data[3];
  aRaw.z = data[4] << 8 | data[5];
  wRaw.x = data[6] << 8 | data[7];
  wRaw.y = data[8] << 8 | data[9];
  wRaw.z = data[10] << 8 | data[11];

  return {aRaw, wRaw};
}

IMUSample MPU6050Sensor::toIMUSample(mpud::raw_axes_t aRaw,
                                     mpud::raw_axes_t wRaw) {
  mpud::float_axes_t aGravity =
      mpud::math::accelGravity(aRaw, MPU6050Sensor::ACCELEROMETER_SCALE);
  mpud::float_axes_t w =
      mpud::math::gyroDegPerSec(wRaw, MPU6050Sensor::GYROSCOPE_SCALE);

  return {.a = {.x = aGravity.x * MPU6050Sensor::g,
                .y = aGravity.y * MPU6050Sensor::g,
                .z = aGravity.z * MPU6050Sensor::g},
          .w = {.roll = w.x, .pitch = w.y, .yaw = w.z},
          .t = esp_timer_get_time()};
}

std::optional<IMUSample> MPU6050Sensor::readSync() {
  mpud::raw_axes_t aRaw, wRaw;
  sensor.acceleration(&aRaw);
  sensor.rotation(&wRaw);
  IMUSample sample = this->toIMUSample(aRaw, wRaw);
  this->logger->info("Sync read: a=<%.3f, %.3f, %.3f> m/s2, "
                     "w=<%.3f, %.3f, %.3f> deg/s, t=%lld us",
                     sample.a.x, sample.a.y, sample.a.z, sample.w.roll,
                     sample.w.pitch, sample.w.yaw, sample.t);
  return sample;
}

void MPU6050Sensor::beginAsync() {
  this->logger->info("Starting async reading");
  sensor.resetFIFO();
  while (this->pipe->pop(false) != std::nullopt) {
    // Empty pipe if any samples are left WITHOUT blocking
  }
  this->setDoRead(true);
}
void MPU6050Sensor::stopAsync() {
  this->logger->info("Stopping async reading");
  this->setDoRead(false);
}
