# Lightweight Kanban

## How to use

1) Write new tasks as S.M.A.R.T-ish goals + changes/artifacts.
2) Classify them either into [Next](#next) (highest priority) or [Backlog](#backlog) (unordered, lower priority).
3) Move tasks to [In Progress](#in-progress) when I start working on them.
4) Move tasks to [Done](#done) when I finish them.
5) When [Next](#next) is empty, move some tasks from [Backlog](#backlog) to [Next](#next), prioritizing them.

Other rules:

- In progress: 1-2 tasks max, avoid multistasking.
- Next: 2-3 tasks max, keep it focused.
- Backlog and Done: unlimited.
- Task duration: break down big tasks into smaller ones, 1-2 days max each.
- History: create PRs and separate branches for non-trivial tasks, self-review them, add link here to PR, and merge directly to main.
- Main: working code. No development branch to keep it simple.

---------------

## Tasks

### In Progress

1. Solder components into perfboard.

### Next

- Figure out how to make a case for the device by 3D printing.
- Design case for device.
- Write final BOM (bill of materials) for the device.

### Backlog

- Review if error macros work with variable arguments
- Fork I2C and MPU libraries on github and apply changes there. Then include as dependencies in the platformIO file.
- Remove unnecessary includes to reduce compiled code size.
- Fine tune transmission parameters to optimize for latency.
- Get raw, real world data from gym exercises: pull ups, dips.
- Learn about filters (complementary, Kalman, etc.) and create a jupyter notebook to experiment with them on fake data.
- Apply filters to real data on jupyter notebook and evaluate results.
- Figure out how to correct drift.
- Figure out how to calibrate the device.
- Implement all math operations on jupyter notebook: from raw data to sanitized velocity signals.
- Implement all math operations on the device: from raw data to sanitized velocity signals.
- Review numerical integration/ODE solving methods and and test them on jupyter notebook with fake data.
- Apply numerical integration/ODE solving methods to real data on jupyter notebook and evaluate results.
- Extract control logic from `src/main.cpp` to separate class.
- Review and update current tasks priorities and ISR behaviors to optimize for latency and responsiveness.
- Read EPS-IDF docs (<https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/ble/index.html#security>) and add basic security to BLE class.
- Move class docs to separate markdown files (documentation as code).
- Create hardware-in-the-loop tests sending commands to the device through serial and verifying readings transmitted through BLE.

### Done

- Rework Logger for less overhead and usefulness: remove atomic sequence number from `Logger` and allow multiple loggers and tags. PR: <https://github.com/FedericoPacheco/gym-imu/pull/1>
- Implement singleton pattern on `MPU6050Sensor`, matching `BLE`'s pattern. Protect both against races. PR: <https://github.com/FedericoPacheco/gym-imu/pull/2>
- Document overall system architecture with C4 and PlantUML: context, containers, components. PR: <https://github.com/FedericoPacheco/gym-imu/pull/3>
- Create basic README for the project: summary, architecture, physical device, next steps, etc. PR: <https://github.com/FedericoPacheco/gym-imu/pull/3>
- Rework Logger for more flexibility: create a `Logger` interface and inherit from the current class renaming it to `UARTLogger`/`SerialLogger`. PR: <https://github.com/FedericoPacheco/gym-imu/pull/4>
- Create test seams for C-interfaces: FreeRTOS, ESP-IDF, NimBLE. PR: <https://github.com/FedericoPacheco/gym-imu/pull/4>
- Create test seams for C++ interfaces: I2C, MPU. PR: <https://github.com/FedericoPacheco/gym-imu/pull/4>
- Create NotificationRunner interface, FreeRTOSNotificationRunner and DeterministicNotificationRunner implementations to simplify testing with multiple FreeRTOS tasks, notifications and ISRs. PR: <https://github.com/FedericoPacheco/gym-imu/pull/4>
- Create LoopRunner interface, FreeRTOSLoopRunner and DeterministicLoopRunner implementations to simplify testing with multiple FreeRTOS tasks that run freely. PR: <https://github.com/FedericoPacheco/gym-imu/pull/4>
- Replace all device libraries imports with ports and runners. PR: <https://github.com/FedericoPacheco/gym-imu/pull/4>
- Create host unit tests for MPU's initialization logic. PR: <https://github.com/FedericoPacheco/gym-imu/pull/4>
- Create host unit tests MPU's readTask logic. PR: <https://github.com/FedericoPacheco/gym-imu/pull/4>
- Create host unit tests for BLE's initialization logic. PR: <https://github.com/FedericoPacheco/gym-imu/pull/4>
- Create host unit tests for BLE's transmitTask logic. PR: <https://github.com/FedericoPacheco/gym-imu/pull/4>
- Rename RETURN_..._ON_ERROR macros to RETURN_..._ON_ESP_ERROR. PR: <https://github.com/FedericoPacheco/gym-imu/pull/4>
- Document testing strategy on README.md. PR: <https://github.com/FedericoPacheco/gym-imu/pull/4>
