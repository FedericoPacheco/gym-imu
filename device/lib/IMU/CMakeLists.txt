idf_component_register(
  SRCS "MPU6050Sensor.cpp" "ICM42688PSensor.cpp" "ICM42688PI2CProbe.cpp"
       "BMI323I2CProbe.cpp"
  INCLUDE_DIRS "."
  REQUIRES driver
)