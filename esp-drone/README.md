# ESP-Drone

ESP-Drone is a project for controlling a drone using an ESP microcontroller.

## Overview

This project provides firmware and tooling to build, flash, and operate a drone powered by an ESP chip. It includes flight controller logic, sensor integration, and wireless communication capabilities.

## Getting Started

### Prerequisites

- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/) (Espressif IoT Development Framework)
- A compatible ESP board (e.g., ESP32)
- USB cable for flashing

### Installation

1. Clone the repository:
    ```bash
    git clone https://github.com/pathofseb/esp-drone.git
    cd esp-drone
    ```

2. Set up your `.env` file (see below).



## Environment Configuration

Create a `.env` file in the project root to configure your build and runtime environment.

### `.env` File Structure

```env
# Wi-Fi Configuration
WIFI_SSID=your_wifi_ssid
WIFI_PASSWORD=your_wifi_password


```

### Variable Reference

| Variable        | Description                              | Required | Default       |
|-----------------|------------------------------------------|----------|---------------|
| `WIFI_SSID`     | Wi-Fi network name for the drone to join | Yes      | —             |
| `WIFI_PASSWORD`  | Wi-Fi network password                  | Yes      | —             |
| `IDF_PATH`      | Path to your ESP-IDF installation        | Yes      | —             |
| `IDF_TARGET`    | Target ESP chip                          | No       | `esp32`       |
| `FLASH_PORT`    | Serial port for flashing                 | No       | `/dev/ttyUSB0`|
| `FLASH_BAUD`    | Baud rate for flashing                   | No       | `115200`      |
| `DRONE_MODE`    | Default flight mode                      | No       | `stabilize`   |
| `LOG_LEVEL`     | Logging verbosity (`debug`, `info`, `warn`, `error`) | No | `info` |

> **Note:** Never commit your `.env` file to version control. Make sure `.env` is listed in your `.gitignore`.

## License

See [LICENSE](LICENSE) for details.