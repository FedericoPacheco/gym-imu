# ADR-id: choose Pipes and Filters architecture style with 3 components and tasks

## Status

Accepted

## Date

2026/02/16

## Context

To date, I've written classes to:

- Control buttons and LEDs
- Perform basic sync and async sensor readings
- Transmit data over the air with BLE in batches

The control flow is in the entrypoint, main.cpp, and I've observed that reading data from there with sensor.readAsync() doesn't allow tight loops, as the core panics when a small wait time is provided to vTaskDelay().

An alternative is for the entrypoint to only instantiate objects and then let them coordinate independently, producing and inserting IMUSamples into an intermediate data structure that can be consumed by a consumer. Currently, those intermediate data structures are MPU6050Sensor and BLE internal queues. This can be generalized to a pipes and filters architecture style, where:

- Producers: starting point of the flow. IMUSensor's concrete classes.
- Transformers: math processing steps: calibration, filtering, sensor fusion, integration, etc.
- Consumers: termination point of the flow. Transmits data to the phone with BLE.
- Pipes: links between steps. FreeRTOS Queues, StreamBuffers (<https://freertos.org/Documentation/02-Kernel/02-Kernel-features/04-Stream-and-message-buffers/01-RTOS-stream-and-message-buffers>) or MessageBuffers hidden behind an interface.

Producers, transformers and consumers can all be independent tasks or grouped together in some form to save resources.

Other options could be to:

- Leave the code relatively as-is, simpler but may be less performant and less maintainable.
- Use Publish/Subscribe with multiple observers and subjects, which is more flexible but isn't strictly necessary for this project due to the simplicity of the data flow.

## Decision

Choose pipes and filters as it fits naturally with my use case and current project structure.
Refactor queues from the sensor and BLE classes to a generic Pipe interface, and implement it with a FreeRTOS queue. Add lightweight metrics such as max depth and drops, as well as a policy to drop oldest items when full. If performance is an issue, switch to a StreamBuffer implementation.
Bundle all math operations in one filter to save resources. Multiple subclasses or the Strategy pattern for each step can still be used.
Use 3 tasks/threads: one for the sensor, one for processing and one for the BLE transmission. If resources are tight, switch to 2 tasks: producer + transformer/consumer.
Acceptance Criteria: packet loss <= 5% at 60Hz for 5 minutes.

## Consequences

### Positive

- Clear separation of concerns and modularity, easier to maintain and extend.
- Each pipe acts as a backpressure mechanism, allowing each step to operate at its own pace without overwhelming the system.

### Negative

- Performance may suffer due to context switching, RAM usage, or inter-task communication overhead, but this can be addressed by later refactors.
