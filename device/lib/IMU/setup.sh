#!/bin/bash

# Download dependencies
cd ../..
mkdir -p components
git clone https://github.com/natanaeljr/esp32-MPU-driver.git MPU
git clone https://github.com/natanaeljr/esp32-I2Cbus.git I2Cbus

# Configure driver
pio run -t menuconfig
# Select: MPU driver:
# > MPU chip model > MPU6050
# > Communication Protocol > I2C
# > Digital Motion Processor (DMP) > Enable
# Press S to save config