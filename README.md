# STM32-to-FPGA Multi-Sensor Monitoring System

A real-time embedded monitoring system combining an STM32H753 microcontroller with a Zybo Z7-20 FPGA for multi-sensor acquisition, local status display, UART communication, hardware alert-state management, and heartbeat fault detection.

## Project Overview

The STM32H753 serves as the primary embedded controller and collects distance, environmental, and motion data from multiple sensors using I2C and SPI.

The STM32 processes sensor data, evaluates alert conditions, updates four OLED displays, and communicates system events to the Zybo Z7-20 FPGA through a custom UART protocol.

The FPGA implements VHDL logic for UART communication, alert-state control, alert-history latching, heartbeat monitoring, and onboard LED status indication.

## Hardware

| Component | Purpose | Interface |
| --- | --- | --- |
| STM32H753ZI | Main embedded controller | — |
| Zybo Z7-20 | FPGA alert and monitoring logic | UART |
| VL53L1X | Time-of-Flight distance sensing | I2C |
| LSM6DS3 | Accelerometer / gyroscope motion sensing | SPI |
| SHT41 | Temperature and humidity sensing | I2C |
| BME280 | Temperature and humidity sensing | I2C |
| PCA/TCA9548A | I2C multiplexer | I2C |
| 4× SSD1306 OLED | Sensor and system status displays | I2C |
| USB-to-TTL Adapter | UART debugging | UART |

## System Architecture

```text
VL53L1X ───── I2C ───────────────► STM32H753
                                      │
LSM6DS3 ───── SPI ────────────────►   │
                                      │
BME280 ──┐                            │
SHT41  ──┤                            │
OLED 1 ──┤                            │
OLED 2 ──┤── I2C MUX ─── I2C ─────► │
OLED 3 ──┤                            │
OLED 4 ──┘                            │
                                      │
                                      │ UART
                                      ▼
                                 Zybo Z7-20
                                  FPGA/VHDL
                                      │
                                      ▼
                                Onboard LEDs

STM32H753 ─── UART Debug ───► USB-to-TTL ───► PC
