# BioSense

BioSense is an experimental biosignal acquisition platform built around an ESP32. The project includes firmware and a custom PCB design. It reads data from several sensors and can send it to a computer or an MQTT broker.

> **Disclaimer:** BioSense is an experimental project, not a medical device. It is not intended for diagnosis, patient monitoring, treatment decisions, or other medical use.

## Features

- ECG acquisition with an AD8232 module, lead-off detection, and heart-rate estimation
- GSR measurement
- MAX30102 heart-rate and SpO₂ estimation
- MLX90614 temperature measurement
- Battery voltage measurement and charge-level estimation

## Hardware

The firmware is configured for the classic ESP32. The KiCad schematics, PCB design: [`hardware/biosense_pcb/`](hardware/biosense_pcb/). See the [hardware README](hardware/biosense_pcb/README.md) for more information about the board and its main components.

The main GPIO assignments: [`main/include/biosense_config.hpp`](main/include/biosense_config.hpp):

## Requirements

- ESP-IDF and the components listed in [`main/idf_component.yml`](main/idf_component.yml)
- The ESP-IDF command-line tools, including `idf.py`
- An ESP32 board and sensors compatible with the project
- An MQTT broker with TLS enabled

## Configuration

Create a local secrets header from the example file.

**PowerShell:**

```powershell
New-Item -ItemType Directory -Force main/private
Copy-Item main/include/biosense_secrets.hpp.example main/private/biosense_secrets.hpp
```

**macOS or Linux:**

```sh
mkdir -p main/private
cp main/include/biosense_secrets.hpp.example main/private/biosense_secrets.hpp
```

Edit `main/private/biosense_secrets.hpp` with your Wi-Fi and service settings.

Set `UDP_TARGET_IP` to the address of the computer that will receive UDP packets. The example uses port `4210`.

For MQTT, configure the broker settings. `MQTT_BROKER_URI` must include a scheme and port, for example:

```text
mqtts://broker.example.com:8883
```

The TLS connection uses the ESP-IDF system certificate bundle.

## Build and flash

Open a terminal configured for ESP-IDF, then run:

```sh
idf.py set-target esp32
idf.py build
idf.py -p PORT flash monitor
```

Replace `PORT` with your board’s serial port, such as `COM3` on Windows or `/dev/ttyUSB0` on Linux. Run `set-target` when setting up the project for the first time or when changing the target chip.

ESP-IDF downloads the required managed components during project configuration or build.

## Data transmission with MQTT

Topics use the `biosense/<MQTT_DEVICE_ID>/` prefix.

| Topic | Contents |
| --- | --- |
| `metrics` | Current measurements and sensor states (JSON) |
| `ecg` | ECG sample batches (JSON) |
| `gsr` | GSR sample batches (JSON) |
| `events` | Button events (JSON) |
| `status` | Device online/offline status |

## Repository layout

```text
main/                   ESP-IDF firmware
  include/              Configuration and module interfaces
  net/                  Wi-Fi, UDP, and MQTT
  sensors/              Sensor acquisition and data conversion
  signal/               ECG signal processing
  io/                   Buttons, buzzer, and LEDs
  power/                Battery-level estimation
hardware/biosense_pcb/  KiCad schematics and PCB design
```

## Security and privacy

Do not commit Wi-Fi passwords, MQTT credentials, or private certificates. Store local credentials in `main/private/biosense_secrets.hpp`.

Biosignal measurements may be sensitive. Secure your broker, network, and data storage accordingly.

## License

The original BioSense firmware and software are licensed under the MIT License. See [LICENSE](LICENSE).

The hardware design files in `hardware/biosense_pcb/` are licensed under CERN-OHL-P-2.0. See the [hardware license](hardware/biosense_pcb/LICENSE).

Third-party software retains its respective license. The MAX3010x algorithms are covered by the license files in [`main/third_party/max3010x_algorithms/`](main/third_party/max3010x_algorithms/).