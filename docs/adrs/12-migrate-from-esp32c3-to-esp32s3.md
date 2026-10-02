# ADR-12: Migrate from ESP32-C3 to ESP32-S3

## Status

Accepted

## Date

2026/10/02

## Context

I'll create a second prototype of the device, and I originally didn't buy a second ESP32 C3.

After the original purchase, I found out the ESP32 C3 doesn't have a FPU (floating point unit). This significantly slowers (roughly an order of magnitude for selected operations, as shown in [1]) mathematical computations that work with real numbers. Representing data as integers may not be adequate for the problem.

The device stores samples on memory, estimates velocity and erases them to make room for new data inmediately after transmission. The samples buffers are limited, and slow computations can lead to data loss. The final goal is to make sampling and processing as high frequency as possible (>>100Hz), which effectively leaves little time budget for computations. Additionally, the microcontroller only has one core, meaning that signal processing, sampling, and transmission share the same processing resources.

Velocity estimation has proven to be difficult to achieve, and even if the IMU is replaced as outlined in ADR-11, more computationally heavy algorithms may be required, namely a variant of the Kalman filter (classic, EKF, UKF, etc.) and some algorithm for detecting phases within repetitions (possibly based on DSP techniques or ML).

After moderate testing, it has been seen that up to 100Hz the device can sample, process and transmit data reliably (time window: 60 seconds). Above this there are cascading failures: multiple interrupts/notifications to read data from the IMU's FIFO queue, the transmission pipe gets full quickly, the watchdog triggers, the recording button stops working. Processing, on the other hand, seems to be able to keep up, at least for the currently implemented operations: noise reduction with causal moving average filter, orientation with simple complementary filter, gravity removal with euler angles, no velocity estimation. Parameters used:

[Constants.hpp](../../device/lib/Constants/include/Constants.hpp):

```cpp
inline constexpr int SAMPLING_FREQUENCY_HZ = 100;

inline constexpr uint32_t SAMPLING_PIPE_SIZE = 128;
inline constexpr uint32_t TRANSMISSION_PIPE_SIZE = SAMPLING_PIPE_SIZE;

inline constexpr int IMU_READ_TASK_STACK_SIZE = 4096;
inline constexpr int IMU_READ_TASK_MAX_BATCH = 12;
inline constexpr int IMU_READ_TASK_PRIORITY = 5;

inline constexpr int PROCESS_TASK_STACK_SIZE = 4096;
inline constexpr int PROCESS_TASK_PRIORITY = 5;

inline constexpr int BLE_TASK_PRIORITY = 4;
inline constexpr int BLE_TASK_STACK_SIZE = 4096;
inline constexpr int TRANSMIT_TASK_PRIORITY = 4;
inline constexpr int TRANSMIT_TASK_STACK_SIZE = 4096;
```

[main.cpp](../../device/src/main.cpp):

```cpp
auto imuRunner = std::make_unique<FreeRTOSNotificationRunner>(
      "readTask", IMU_READ_TASK_STACK_SIZE, IMU_READ_TASK_PRIORITY);
// ...
auto processorRunner = std::make_unique<FreeRTOSLoopRunner>(
      "processTask", PROCESS_TASK_STACK_SIZE, PROCESS_TASK_PRIORITY,
      pdMS_TO_TICKS(5));
// ...
auto bleLoopRunner = std::make_unique<FreeRTOSLoopRunner>(
      "transmitTask", TRANSMIT_TASK_STACK_SIZE, TRANSMIT_TASK_PRIORITY,
      pdMS_TO_TICKS(50));
```

Given this context, although the ESP32 C3 can handle the current workload, migrating to a microcontroller with a FPU within the Espressif ecosystem may provide more breathing room for future development. The following options have been considered:

| Feature | [ESP32-C3](<https://www.espressif.com/en/products/socs/esp32-c3>) (current) | [ESP32-C6](<https://www.espressif.com/en/products/socs/esp32-c6>) | [ESP32-S3](<https://www.espressif.com/en/products/socs/esp32-s3>) | [ESP32-H4](<https://www.espressif.com/en/products/socs/esp32-h4>) | [ESP32-P4](<https://www.espressif.com/en/products/socs/esp32-p4>) |
| -- | -- | -- | -- | -- | -- |
| Summary | Cost-effective SoC | Cost-effective SoC with increased connectivity | High-performance SoC that supports DSP/AI workloads | Low-power SoC with increased connectivity | Powerhouse SoC with advanced features for images/video, GUIs, etc. |
| ISA | 32 bit RISC-V | 32 bit RISC-V | 32 bit Xtensa LX7 | 32 bit RISC-V | 32 bit RISC-V |
| CPUs | 1x 160 MHz | 1x 160 MHz + 1x 20 MHz | 2x 240 MHz | 2x 96 MHz | 2x 400 MHz + 1x 40 MHz |
| FPU (single precision) | No | No | Yes | Yes | Yes |
| (Board) memory | 400 KB SRAM | 512 KB SRAM | 512 KB SRAM | 384 KB SRAM | 768 KB SRAM + 8 KB TCM RAM |
| (Board) flash | 4 MB | 4/8/16 MB | 8/16 MB | 4 MB | 16/32 MB |
| Communications | Wi-Fi, Bluetooth 5 (LE) | Wi-Fi 6, Bluetooth 5 (LE), Thread 1.3, Zigbee 3.0 | 802.11 b/g/n Wi-Fi, Bluetooth 5 (LE) | Bluetooth 5.4 (LE), Thread 1.4, Zigbee 3.0 | Wi-Fi 6, Thread/Zigbee |

As can be seen in [4], only the C3, C6 and S3  are available in the Seeed Studio XIAO series (thumb-sized form factor with battery module I've used for the first prototype). All three options are available at Mercado Libre at ~13USD, ~14USD, and ~24USD respectively, without considering shipping.

Migrating to other microcontrollers (e.g. STM32, Nordic nRF series) would likely require significant changes to the software stack, which is not desirable at this stage of the project.

Sources:

1. [Floating-Point Units on Espressif SoCs: Why (and when) they matter](https://developer.espressif.com/blog/2025/10/cores_with_fpu/)
2. <https://docs.espressif.com/projects/esp-idf/en/stable/esp32c3/api-guides/performance/speed.html#improving-overall-speed>
3. <https://www.reddit.com/r/esp32/comments/1la8fob/esp32_floating_point_performance/>
4. <https://wiki.seeedstudio.com/SeeedStudio_XIAO_Series_Introduction/>
5. <https://wiki.seeedstudio.com/XIAO_ESP32C3_Getting_Started/>
6. <https://wiki.seeedstudio.com/XIAO_ESP32C6_Getting_Started/>
7. <https://wiki.seeedstudio.com/XIAO_ESP32S3_Getting_Started/>

## Decision

Choose the Seeedstudio XIAO ESP32-S3 for the next iteration of the project, as it offers a higher performance for mathematical operations due to its single-precision FPU, it isn't prohibitively more expensive, remains within the ESP32 ecosystem, and the ISA migration will not have application-level impact (since the codebase doesn't have any assembly code), but rather just some schematic and configuration changes.

Experiment with optimized routines from the [ESP-DSP](https://docs.espressif.com/projects/esp-dsp/en/latest/esp32/index.html) library and assigning one fixed core to handle DSP tasks. Also consider using a slightly bigger battery (e.g. 800mAh) with the same form factor as the previous one to accomodate for additional power consumption in case it's proven to be needed.

## Consequences

### Positive

* More RAM, CPU performance, and storage capacity compared to the ESP32-C3 for future development.

### Negative

* Higher power consumption compared to the ESP32-C3, which may require a larger battery or more frequent charging.

* Slightly more expensive compared to the ESP32-C3, but this isn't problematic since the final product will be a one-off or low-volume project.
