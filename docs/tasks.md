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

- Consider merging GPIO functions in ESPCompatibility.hpp.
- Add essential unit tests for `MPU` class readTask logic.

### Next

1. Add essential unit tests for `BLE` class transmitTask logic.

### Backlog

- Add checks for logger null pointers across all classes
- Review if error macros work with variable arguments
- Solder components into perfboard.
- Figure out how to make a case for the device (3D printing?) and design it.
- Write final BOM (bill of materials) for the device.
- Fork I2C and MPU libraries and apply changes there. Then include as dependencies in the platformIO file.
- Remove unnecessary includes to reduce compiled code size.
- Rename RETURN_..._ON_ERROR macros to RETURN_..._ON_ESP_ERROR.
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

### Done

- Rework Logger for less overhead and usefulness: remove atomic sequence number from `Logger` and allow multiple loggers and tags. PR: <https://github.com/FedericoPacheco/gym-imu/pull/1>
- Implement singleton pattern on `MPU6050Sensor`, matching `BLE`'s pattern. Protect both against races. PR: <https://github.com/FedericoPacheco/gym-imu/pull/2>
- Document overall system architecture with C4 and PlantUML: context, containers, components. PR: <https://github.com/FedericoPacheco/gym-imu/pull/3>
- Create basic README for the project: summary, architecture, physical device, next steps, etc. PR: <https://github.com/FedericoPacheco/gym-imu/pull/3>
- Rework Logger for more flexibility: create a `Logger` interface and inherit from the current class renaming it to `UARTLogger`/`SerialLogger`
- Move all platform(esp, freertos, etc.) ports to include and my own ones to lib.
- Create compatibility file for FreeRTOSPort.
