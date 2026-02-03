#include "MPU6050Sensor.hpp"
#include "I2Cbus.hpp"
#include "IMUSensor.hpp"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "mpu/math.hpp"
#include <cstdint>
#include <memory>

MPU6050Sensor::MPU6050Sensor(Logger *logger, gpio_num_t INTPin,
                             gpio_num_t SDAPin, gpio_num_t SCLPin,
                             int samplingFrequencyHz)
    : logger(logger), bus(I2C_NUM_0) {

  this->logger->debug("Sensor parameters: INTPin=%d, SDAPin=%d, SCLPin=%d, "
                      "samplingFrequencyHz=%d",
                      INTPin, SDAPin, SCLPin, samplingFrequencyHz);

  this->INTPin = INTPin;
  this->SDAPin = SDAPin;
  this->SCLPin = SCLPin;
  this->samplingFrequencyHz = samplingFrequencyHz;
}

std::unique_ptr<IMUSensor>
MPU6050Sensor::create(Logger *logger, gpio_num_t INTPin, gpio_num_t SDAPin,
                      gpio_num_t SCLPin, int samplingFrequencyHz) {

  std::unique_ptr<MPU6050Sensor> imu(
      new MPU6050Sensor(logger, INTPin, SDAPin, SCLPin, samplingFrequencyHz));

  if (!initializeI2CBus(imu.get()))
    return nullptr;
  if (!resetSensor(imu.get()))
    return nullptr;
  // performDiagnostics(imu.get());
  if (!testConnection(imu.get()))
    return nullptr;
  if (!initializeSensor(imu.get()))
    return nullptr;
  configureSettings(imu.get());
  setupDMPQueue(imu.get());
  configureInterrupts(imu.get());
  setupAsyncComponents(imu.get());

  imu->logger->info("MPU6050 sensor initialized successfully");

  return imu;
}

bool MPU6050Sensor::initializeI2CBus(MPU6050Sensor *imu) {
  imu->logger->debug("Initializing I2C bus");
  imu->bus.begin(imu->SDAPin, imu->SCLPin, GPIO_PULLUP_DISABLE,
                 GPIO_PULLUP_DISABLE, MPU6050Sensor::BUS_FREQUENCY_HZ);
  imu->sensor.setBus(imu->bus);
  imu->sensor.setAddr(mpud::MPU_I2CADDRESS_AD0_LOW);
  return true;
}

bool MPU6050Sensor::resetSensor(MPU6050Sensor *imu) {
  imu->logger->debug("Performing sensor reset");
  esp_err_t err = imu->sensor.reset();
  if (err != ESP_OK) {
    imu->logger->error("Reset failed: %s", esp_err_to_name(err));
    return false;
  }
  vTaskDelay(pdMS_TO_TICKS(250));
  return true;
}

void MPU6050Sensor::performDiagnostics(MPU6050Sensor *imu) {
  imu->logger->debug("Scanning I2C bus for MPU6050 device");
  imu->bus.scanner();

  uint8_t wai = imu->sensor.whoAmI();
  imu->logger->debug("Retrieved sensor's WHO_AM_I register: 0x%02X", wai);

  imu->logger->debug("Retrieved sensor's register dump (0x00..0x1F):");
  imu->sensor.registerDump(0x00, 0x1F);

  mpud::selftest_t selfTestResult = 0;
  esp_err_t serr = imu->sensor.selfTest(&selfTestResult);
  imu->logger->info("selfTest result: 0x%X err=%s", selfTestResult,
                    esp_err_to_name(serr));
}

bool MPU6050Sensor::testConnection(MPU6050Sensor *imu) {
  imu->logger->debug("Attempting I2C connection");
  uint8_t attempts = 0;
  uint8_t MAX_CONNECTION_ATTEMPTS = 5;
  esp_err_t err = imu->sensor.testConnection();
  while (err != ESP_OK && attempts < MAX_CONNECTION_ATTEMPTS) {
    imu->logger->error("I2C connection failed: %s (%#X)", esp_err_to_name(err),
                       err);
    vTaskDelay(pdMS_TO_TICKS(1000));
    err = imu->sensor.testConnection();
    attempts++;
  }
  if (attempts == MAX_CONNECTION_ATTEMPTS) {
    imu->logger->error("I2C connection failed after %d attempts", attempts);
    return false;
  }
  imu->logger->debug("I2C connection successful");
  return true;
}

bool MPU6050Sensor::initializeSensor(MPU6050Sensor *imu) {
  imu->logger->debug("Initializing MPU6050 sensor");
  esp_err_t err = imu->sensor.initialize();
  if (err != ESP_OK) {
    imu->logger->error("MPU initialization failed: %s (%#X)",
                       esp_err_to_name(err), err);
    return false;
  }
  return true;
}

void MPU6050Sensor::configureSettings(MPU6050Sensor *imu) {
  imu->logger->debug("Configuring sensor settings");
  imu->sensor.setSampleRate(imu->samplingFrequencyHz);
  imu->sensor.setAccelFullScale(MPU6050Sensor::ACCELEROMETER_SCALE);
  imu->sensor.setGyroFullScale(MPU6050Sensor::GYROSCOPE_SCALE);
}

void MPU6050Sensor::setupDMPQueue(MPU6050Sensor *imu) {
  imu->logger->debug("Setting up DMP FIFO queue on sensor");
  imu->sensor.setFIFOConfig(mpud::FIFO_CFG_ACCEL | mpud::FIFO_CFG_GYRO);
  imu->sensor.setFIFOEnabled(true);
}

void MPU6050Sensor::configureInterrupts(MPU6050Sensor *imu) {
  imu->logger->debug("Configuring pin and interrupts");
  const gpio_config_t pinConfig{.pin_bit_mask = (uint64_t)0x1 << imu->INTPin,
                                .mode = GPIO_MODE_INPUT,
                                .pull_up_en = GPIO_PULLUP_DISABLE,
                                .pull_down_en = GPIO_PULLDOWN_ENABLE,
                                .intr_type = GPIO_INTR_POSEDGE};
  gpio_config(&pinConfig);
  gpio_isr_handler_add(imu->INTPin, MPU6050Sensor::isrHandler, imu);
  const mpud::int_config_t intConfig{.level = mpud::INT_LVL_ACTIVE_HIGH,
                                     .drive = mpud::INT_DRV_PUSHPULL,
                                     .mode = mpud::INT_MODE_PULSE50US,
                                     .clear = mpud::INT_CLEAR_STATUS_REG};
  // gpio_install_isr_service() assumed to be called from app_main()
  imu->sensor.setInterruptConfig(intConfig);
  imu->sensor.setInterruptEnabled(mpud::INT_EN_RAWDATA_READY);
}

void MPU6050Sensor::setupAsyncComponents(MPU6050Sensor *imu) {
  imu->logger->debug(
      "Setting up FreeRTOS task and static queue for async reading");
  imu->setDoRead(false);
  xTaskCreate(readTask, "readTask", MPU6050Sensor::READ_TASK_STACK_SIZE, imu,
              MPU6050Sensor::READ_TASK_PRIORITY, &imu->readTaskHandle);
  imu->sampleQueue =
      xQueueCreateStatic(MPU6050Sensor::SAMPLE_QUEUE_SIZE, sizeof(IMUSample),
                         imu->sampleQueueBuffer, &imu->sampleQueueControlBlock);
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

  this->logger->debug("Deleting sample queue");
  if (this->sampleQueue) {
    vQueueDelete(this->sampleQueue);
  }

  this->logger->info("MPU6050 sensor shut down successfully");
}

void MPU6050Sensor::setDoRead(bool value) {
  this->doRead.store(value, std::memory_order_relaxed);
}
bool MPU6050Sensor::getDoRead() {
  return this->doRead.load(std::memory_order_relaxed);
}

std::optional<IMUSample> MPU6050Sensor::readAsync() {
  IMUSample sample;
  if (xQueueReceive(this->sampleQueue, &sample, portMAX_DELAY) == pdPASS) {
    this->logger->info("Async read: a=<%.3f, %.3f, %.3f> m/s2, "
                       "w=<%.3f, %.3f, %.3f> deg/s, t=%lld us",
                       sample.a.x, sample.a.y, sample.a.z, sample.w.roll,
                       sample.w.pitch, sample.w.yaw, sample.t);
    return sample;
  }
  this->logger->warn("Async read: failed");
  return std::nullopt;
}

void IRAM_ATTR MPU6050Sensor::isrHandler(void *arg) {
  MPU6050Sensor *self = static_cast<MPU6050Sensor *>(arg);

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
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (!self->getDoRead()) {
      continue;
    }

    uint16_t fifoCount = self->sensor.getFIFOCount();
    bool wasCountReadError = self->sensor.lastError() != ESP_OK;
    bool isFifoMisaligned = fifoCount % MPU6050Sensor::FIFO_PACKET_SIZE != 0;
    bool isOnePacketAvailable = fifoCount >= MPU6050Sensor::FIFO_PACKET_SIZE &&
                                fifoCount < 2 * MPU6050Sensor::FIFO_PACKET_SIZE;
    bool areManyPacketsAvailable =
        fifoCount > 2 * MPU6050Sensor::FIFO_PACKET_SIZE;

    if (wasCountReadError) {
      self->logger->warn("Read task: FIFO count read error");
      self->sensor.resetFIFO();
      continue;
    }
    if (isFifoMisaligned) {
      self->logger->warn("Read task: FIFO misaligned (count=%d)", fifoCount);
      self->sensor.resetFIFO();
      continue;
    }
    if (areManyPacketsAvailable) {
      self->logger->warn("Read task: many packets available (count=%d)",
                         fifoCount);
      self->sensor.resetFIFO();
      continue;
    }
    if (!isOnePacketAvailable) {
      continue;
    }

    uint8_t sensorBuffer[MPU6050Sensor::FIFO_PACKET_SIZE];
    bool wasBufferReadError =
        self->sensor.readFIFO(MPU6050Sensor::FIFO_PACKET_SIZE, sensorBuffer) !=
        ESP_OK;
    if (wasBufferReadError) {
      self->logger->warn("Read task: FIFO buffer read error");
      self->sensor.resetFIFO();
      continue;
    }

    if (isOnePacketAvailable) {
      self->logger->debug("Read task: one packet available (count=%d)",
                          fifoCount);
      auto [aRaw, wRaw] = self->parseSensorData(sensorBuffer);

      IMUSample sample = self->convertToSample(aRaw, wRaw);
      self->logger->debug(
          "Read task: enqueueing sample: a=<%.3f, %.3f, %.3f> m/s2, "
          "w=<%.3f, %.3f, %.3f> deg/s, t=%lld us",
          sample.a.x, sample.a.y, sample.a.z, sample.w.roll, sample.w.pitch,
          sample.w.yaw, sample.t);
      xQueueSend(self->sampleQueue, &sample, portMAX_DELAY);
    }
  }

  self->logger->debug("Read task ended");
}

std::tuple<mpud::raw_axes_t, mpud::raw_axes_t>
MPU6050Sensor::parseSensorData(const uint8_t *data) {
  mpud::raw_axes_t aRaw, wRaw;

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

IMUSample MPU6050Sensor::convertToSample(mpud::raw_axes_t aRaw,
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
  IMUSample sample = this->convertToSample(aRaw, wRaw);
  this->logger->info("Sync read: a=<%.3f, %.3f, %.3f> m/s2, "
                     "w=<%.3f, %.3f, %.3f> deg/s, t=%lld us",
                     sample.a.x, sample.a.y, sample.a.z, sample.w.roll,
                     sample.w.pitch, sample.w.yaw, sample.t);
  return sample;
}

void MPU6050Sensor::beginAsync() {
  this->logger->info("Starting async reading");
  sensor.resetFIFO();
  IMUSample sample;
  while (xQueueReceive(this->sampleQueue, &sample, 0) == pdPASS) {
    // Empty queue if any samples are left WITHOUT blocking (pdPASS)
  }
  this->setDoRead(true);
}
void MPU6050Sensor::stopAsync() {
  this->logger->info("Stopping async reading");
  this->setDoRead(false);
}