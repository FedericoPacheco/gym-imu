# ADR-6: choose NimBLE library for wireless communication

## Status

Accepted

## Date

2026/02/05

## Context

I've found two main options for wireless communication on ESP32:

- NimBLE: BLE only, modern/clean API, modular, lower footprint (memory, energy).
- Bluedroid: BLE + Bluetooth classic, older/legacy but more madure, monolithic, slightly higher performance historically, higher footprint.

My ESP32 c3 only supports BLE.

The ESP-IDF repo has examples for both libraries (<https://github.com/espressif/esp-idf/tree/master/examples/bluetooth>), but the online docs only seems to cover NimBLE (<https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/ble/index.html#get-started>).

## Decision

Choose NimBLE over Bluedroid as it is the default for new ESP32 projects and fits exactly my communication needs.

## Consequences

### Positive

Listed on Context section.

### Negative

No major downsides.
