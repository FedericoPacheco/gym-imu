idf_component_register(
  SRCS "VBTSystem.cpp"
  INCLUDE_DIRS "."
  REQUIRES BLE Button LED IMU SignalProcessing Pipe Logger
)
