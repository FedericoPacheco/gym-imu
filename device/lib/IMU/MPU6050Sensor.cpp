#include "MPU6050Sensor.hpp"
#include "I2Cbus.hpp"
#include "IMUSensor.hpp"
#include "esp_timer.h"
#include "freertos/idf_additions.h"
#include <array>
#include <cstdint>
#include <iostream>

MPU6050Sensor::MPU6050Sensor(gpio_num_t INTPin, gpio_num_t SDAPin,
                             gpio_num_t SCLPin, int samplingFrequencyHz) {
  this->INTPin = INTPin;
  this->SDAPin = SDAPin;
  this->SCLPin = SCLPin;
  this->samplingFrequencyHz = samplingFrequencyHz;

  // Configure I2C bus
  i2c0.begin(this->SDAPin, this->SCLPin, MPU6050Sensor::BUS_FREQUENCY_HZ);
  sensor.setBus(i2c0);
  sensor.setAddr(mpud::MPU_I2CADDRESS_AD0_LOW);
  esp_err_t err = sensor.testConnection();
  if (err != ESP_OK) {
    std::cout << "MPU connection failed:" << esp_err_to_name(err) << std::endl;
  }

  // Initialize and configure sensor settings
  err = sensor.initialize();
  if (err != ESP_OK) {
    std::cout << "MPU init failed:" << esp_err_to_name(err) << std::endl;
  }
  sensor.setSampleRate(this->samplingFrequencyHz);
  sensor.setAccelFullScale(MPU6050Sensor::ACCELEROMETER_SCALE);
  sensor.setGyroFullScale(MPU6050Sensor::GYROSCOPE_SCALE);

  // Setup FIFO queue on sensor
  sensor.setFIFOConfig(mpud::FIFO_CFG_ACCEL | mpud::FIFO_CFG_GYRO);
  sensor.setFIFOEnabled(true);

  // Configure pin and interrupts
  const gpio_config_t pinConfig{.pin_bit_mask = (uint64_t)0x1 << this->INTPin,
                                .mode = GPIO_MODE_INPUT,
                                .pull_up_en = GPIO_PULLUP_DISABLE,
                                .pull_down_en = GPIO_PULLDOWN_ENABLE,
                                .intr_type = GPIO_INTR_POSEDGE};
  gpio_config(&pinConfig);
  gpio_isr_handler_add(this->INTPin, MPU6050Sensor::isrHandler, this);
  const mpud::int_config_t intConfig{.level = mpud::INT_LVL_ACTIVE_HIGH,
                                     .drive = mpud::INT_DRV_PUSHPULL,
                                     .mode = mpud::INT_MODE_PULSE50US,
                                     .clear = mpud::INT_CLEAR_STATUS_REG};
  // gpio_install_isr_service() assumed to be called from app_main()
  sensor.setInterruptConfig(intConfig);
  sensor.setInterruptEnabled(mpud::INT_EN_RAWDATA_READY);

  // Configure FreeRTOS task and
  // static queue (no dynamic allocation) for async reading
  this->setDoRead(false);
  xTaskCreate(readTask, "readTask", MPU6050Sensor::READ_TASK_STACK_SIZE, this,
              MPU6050Sensor::READ_TASK_PRIORITY, &this->readTaskHandle);
  this->sampleQueue = xQueueCreateStatic(
      MPU6050Sensor::SAMPLE_QUEUE_SIZE, sizeof(IMUSample),
      this->sampleQueueBuffer, &this->sampleQueueControlBlock);
}

MPU6050Sensor::~MPU6050Sensor() {
  this->setDoRead(false);

  // Notify the task first to unblock and finish cleanly, then delete
  if (this->readTaskHandle) {
    vTaskNotifyGive(this->readTaskHandle);
    vTaskDelay(1);
    vTaskDelete(this->readTaskHandle);
  }

  gpio_isr_handler_remove(this->INTPin);

  if (this->sampleQueue) {
    vQueueDelete(this->sampleQueue);
  }
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
    return sample;
  }
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
    if (wasCountReadError || areManyPacketsAvailable || isFifoMisaligned) {
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
      self->sensor.resetFIFO();
      continue;
    }

    if (isOnePacketAvailable) {
      auto [aRaw, wRaw] = self->parseSensorData(sensorBuffer);

      IMUSample sample = self->convertToSample(aRaw, wRaw);
      xQueueSend(self->sampleQueue, &sample, portMAX_DELAY);
    }
  }
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
      mpud::accelGravity(aRaw, MPU6050Sensor::ACCELEROMETER_SCALE);
  mpud::float_axes_t w =
      mpud::gyroDecPerSec(wRaw, MPU6050Sensor::GYROSCOPE_SCALE);

  return {.a = {.x = aGravity.x / MPU6050Sensor::G,
                .y = aGravity.y / MPU6050Sensor::G,
                .z = aGravity.z / MPU6050Sensor::G},
          .w = {.roll = w.x, .pitch = w.y, .yaw = w.z},
          .t = esp_timer_get_time()};
}

std::optional<IMUSample> MPU6050Sensor::readSync() {
  mpud::raw_axes_t aRaw, wRaw;
  sensor.acceleration(&aRaw);
  sensor.rotation(&wRaw);

  return this->convertToSample(aRaw, wRaw);
}

void MPU6050Sensor::beginAsync() {
  sensor.resetFIFO();
  IMUSample sample;
  while (xQueueReceive(this->sampleQueue, &sample, 0) == pdPASS) {
    // Empty queue if any samples are left WITHOUT blocking (pdPASS)
  }
  this->setDoRead(true);
}
void MPU6050Sensor::stopAsync() { this->setDoRead(false); }