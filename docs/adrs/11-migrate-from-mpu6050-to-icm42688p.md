# ADR-11: Migrate from MPU6050 to ICM-42688-P

## Status

Accepted

## Date

2026/09/28

## Context

### Experience with current sensor

I'm currently using a [MPU6050](../datasheets/MPU6050%20datasheet.pdf), and I've experienced the following issues after working on velocity estimation:

* The sensor is a clone/counterfeit, so performance is probably worse than a genuine MPU6050, which has been discontinued many years ago.
* The genuine sensor itself prioritizes low cost and general-purpose usage over high-precision.
* The accelerometer has significant bias, about a 0.4 m/s^2 difference with respect to gravity when stationary in the worst case.
* The gyroscope has a significant bias, about 5°/s when stationary in the worst case.
* The accelerometer has significant noise, with a standard deviation of about 0.03 m/s^2 in the worst case when stationary.
* Using a Mahony filter for orientation, after gravity removal, the residual acceleration is of about 0.1-0.2 m/s^2 in the worst case even for stationary captures.
* For exercise captures, velocity drifts within 10-15 s max.
* For exercise captures, velocity values are unreasonable, 10 m/s=36 km/h in the worst case after less than 60s of motion.

There are possible higher-effort improvements for my processing pipeline, namely using a Kalman filter variant for orientation estimation and working on shortening integration windows to concentric intervals only for velocity computation, but the sensor doesn't help.

### Economic situation

I have a very modest income from my teacher assistant role at university, and can afford to spend at most 50-60 USD on electronics components per month without using savings. Based on past experience, it's better to buy 2 units of each component ideally to address manufacturing defects or my own mistakes during assembly and testing.

### Sensor research

I have to make a second version of the prototype anyways. For this reason, I researched new sensors online. There are a variety of local vendors, but they mostly sell components closely related to the Arduino ecosystem and general-purpose electronics instead of what I'm looking for:

* [Emakers](https://www.emakers.com.ar/)
* [GMElectronica](https://gmelectronica.com.ar/)
* [Electrocomponentes](https://www.electrocomponentes.com/)
* [Microelectronicash](https://www.microelectronicash.com/)
* [Candy-ho](https://candy-ho.com/)

The "professional" suppliers sell standalone ICs and components, rather than ready-to-use modules:

* [Mouser](https://www.ar.mouser.com/): international, wider range of components.
* [Elemon](https://www.elemon.com.ar/): local distributor, lower stock availability.

This makes assembly significantly more complex.

Because of this, I'll just stick to general-purpose suppliers like Mercado Libre, which now has the option of buying directly to China. TEMU doesn't sell electronic components. Tienda Mia offers them but has higher prices in general.

I'll also stick to MEMS technology since that's what generally available from suppliers and they fit my budget and size constraints.

#### High performance

Fit the main need of the project: give accurate and reliable readings.

* [*TDK ICM-45686*](../datasheets/ds-000577-icm-45686-datasheet.pdf): not available.
* [*ST LSM6DSV16X*](../datasheets/DS_lsm6dsv16x.pdf): not available.
* [*Analog Devices ADIS16507*](../datasheets/ADIS16507.pdf): not available.
* [*ICM-42688-P*](../datasheets/DS-000347%20ICM-42688-P%20v1.9.pdf): about 17 USD/each (conversion: 1USD = 1500ARS).

#### Industrial applications

High reliability under harsh conditions (temperature, vibration, and shock). Not necessary considering a typical gym environment with moderate temperatures (10° C to 30° C) and regular levels of vibration and shock.

* [*TDK IIM-42653*](../datasheets/ds-000529-iim-42653-datasheet.pdf): not available.
* [*ST ISM330DHCX*](../datasheets/DS_ism330dhcx.pdf): about 68USD/each.

#### General purpose, low power, or with magnetometer

The magnetometer can be used for computing absolute heading, but this is probably unnecessary for VBT and it may be affected due to ferromagnetic gym equipment, which alters the local magnetic field. "General purpose" may mean that the sensor does not meet the accuracy requirements for high-performance motion tracking. The Bosch BNO family specifically offers built-in vendor-specific sensor fusion. While convenient, it limits flexibility to apply custom algorithms.

* [*TDK ICM-20948*](../datasheets/ds-000189-icm-20948-datasheet.pdf): about 19USD/each.
* [*Bosch BMI270*](../datasheets/bst-bmi270-ds000.pdf): not available.
* [*Bosch BMI323*](../datasheets/bst-bmi323-ds000.PDF): about 18 USD/each.
* [*Bosch BNO055*](../datasheets/bst-bno055-ds000.pdf): about 36 USD/each.
* [*Bosch BNO080/BNO085*](../datasheets/BNO080_085-Datasheet.pdf): about 40 USD/each.
* [*ST LSM6DS3*](../datasheets/st_imu_lsm6ds3_datasheet.pdf): about 41 USD/each.

Other sources:

* <https://www.bettlink.com/blog/mpu6050-alternatives>

## Decision

Choose the ICM-42688-P due to its relatively low cost, higher availability and improved performance metrics, as detailed in the comparison below (values retrieved from the respective datasheets):

**Gyroscope**:

| Metric | MPU-6050 | ICM-42688-P |
| --- | ---: | ---: |
| Scales | ±250/500/1000/±2000 °/s | ± 15.625/31.25/62.5/125/250/500/1000/2000 °/s |
| Sensitivity Scale Factor Initial Tolerance | ±3 % | ±0.5 % |
| Sensitivity scale factor variation over temperature | ±2 % | ±0.005 %/°C (0 to +70 °C) |
| Non-linearity | 0.2 % | ±0.1 % |
| Cross-axis sensitivity | ±2 % | ±1.25 % |
| Zero rate output (ZRO) tolerance (bias) | ±20 °/s | ±0.5 °/s |
| ZRO variation over temperature | ±20 °/s (-40 to +85 °C) | ±0.005 °/s/°C (0 to +70 °C) |
| Noise spectral density | 0.005 °/s/√Hz | 0.0028 °/s/√Hz |
| Resolution | 16-bit | 16/19-bit |

**Accelerometer**:

| Metric | MPU-6050 | ICM-42688-P |
| --- | ---: | ---: |
| Scales | ±2/4/8/16 g | ±2/4/8/16 g |
| Sensitivity scale factor initial tolerance | ±3 % | ±0.5 % |
| Sensitivity scale factor variation over temperature | ±0.02 %/°C (-40 to +85 °C) | ±0.005 %/°C (0 to +70 °C) |
| Non-linearity | ±0.5 % | ±0.1 % |
| Initial zero-g tolerance | X, Y: ±50 mg; Z: ±80 mg | X, Y, Z: ±20 mg |
| Zero-G level change over temperature | X, Y: ±35 mg; Z: ±60 mg (0 to +70 °C) | X, Y, Z: ±0.15 mg/°C (-40 to +85 °C) |
| Noise spectral density | 400 µg/√Hz | X, Y: 65 µg/√Hz; Z: 70 µg/√Hz |
| Accelerometer sensitivity tolerance | ±3 % | ±0.5 % |
| Resolution | 16-bit | 16/18-bit |

**Others**:

| Metric | MPU-6050 | ICM-42688-P |
| --- | ---: | ---: |
| FIFO | 1 KB | 2 KB |
| Host interface (communication) | 400 kHz I2C | 12.5 MHz I3C, 1 MHz I2C, 24 MHz SPI |
| Timestamps | No | Yes |
| External clock input | No | Yes |
| Low pass filters | Yes | Yes |
| Temperature sensor | Yes | Yes |
| Motion features | Pedometer, support for gesture recognition, panning, zooming, scrolling, taps, shaking | Pedometer, tilt detection, tap detection, wake on motion, significant motion detection |

The ICM-42688-P improves many sources of errors (bias, noise, non-linearity, cross-axis sensitivity, variation over temperature), has better sensitivity, and offers timestamps. All these features help with better velocity estimation.

Regarding the device driver for the sensor, there are a couple of options:

* <https://github.com/VaderMester/ESP32-ICM426XX-driver>: 5 years without updates. Only tested with ESP-IDF 4.4, I2C and an ICM-42605 sensor. No releases.
* <https://github.com/Barsy-Barsevich/ICM42688_Barsotion>: almost non-existent documentation + spare comments in russian. For some reason has code referring to Kalman and the FFT. No releases. Specific to my sensor. Its FIFO subroutine appears to use an SPI-specific read routine despite also offering I2C support.
* <https://github.com/nickchen110/ICM42688_SensorAPI>: maybe the cleanest but very incomplete (I2C only, raw reads, config, power down, no FIFO support), relatively recent activity (8 months ago). Released on the ESP registry: <https://components.espressif.com/components/nickchen110/icm42688/versions/1.0.3/readme>.

All options, in case they are used, will probably require a more thorough code review, adaptation and testing. Because of this, I'll attempt to write a custom driver tailored to the specific needs, having the previous drivers and the sensor datasheet as reference. The rest of the system should remain unchanged provided the code implements the IMUSensorPort adapter and streams samples to queue connecting to the task with the IMUSignalProcessor instance.

After writing the driver, assemble a new version of the prototype and carry it to the gym to record a new set of IMU data. These captures can later be analyzed with the existent offline processing jupyter notebooks to address the accuracy improvements.

## Consequences

### Positive

* Better measurements give more margin to achieve a better velocity estimation.
* I'll have the educational experience of writing my first custom driver, and I avoid using bad third-party code.

### Negative

* Embedded development can be challenging and time-consuming, but this can be mitigated with incremental development and testing and by reducing the feature set to the essential requirements: initialization and basic configuration (scales, low pass filters, etc.), reading data directly from registers through SPI, reading data from FIFO buffer on interrupt.
* I may change the sensor and still be far from achieving accurate velocity estimation, but I already have other signal processing ideas to explore later and an incremental improvement can also be valuable.
