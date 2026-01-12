# arduino_serial

ROS Noetic package for bidirectional serial communication between an Arduino and ROS.

This package is designed for **robust, low-level I/O integration** in robotic systems, where an Arduino handles ADC acquisition and digital outputs, and ROS manages supervision, logic, and integration with the rest of the system.

---

## Features

- Serial communication using a **JSON-based protocol**
- ADC acquisition from Arduino:
  - Analog pins: `A0`, `A1`, `A2`, `A3`
- Control of **two digital outputs** on Arduino:
  - Default pins: `D8`, `D9`
  - Safe startup state: **all outputs LOW**
- Runtime configuration from ROS:
  - Change ADC publishing period (`period_ms`)
  - Enable/disable digital outputs via ROS services
- Designed to be:
  - Simple
  - Debuggable
  - Easy to extend

---

## Package Structure

```
arduino_serial/
├── CMakeLists.txt
├── package.xml
├── README.md
├── scripts/
│   └── serial_to_ros.py        # ROS node (Python)
├── launch/
│   └── arduino_serial.launch   # Launch file
└── firmware/
    └── arduino_serial.ino      # Arduino firmware (stored for versioning)
```

> ⚠️ The Arduino firmware is **not compiled by ROS**.  
> It is stored here for version control and traceability.

---

## Serial Protocol

### Arduino → ROS (ADC data)

Published periodically as a single JSON line:

```json
{"A0":123,"A1":456,"A2":789,"A3":321}
```

### ROS → Arduino (commands)

#### Set digital output

```json
{"cmd":"set_output","pin":8,"value":1}
```

```json
{"cmd":"set_output","pin":9,"value":0}
```

#### Change ADC period

```json
{"cmd":"set_period","period_ms":100}
```

---

## ROS Interfaces

### Topics

- **`/arduino/adc`** (`std_msgs/Int32MultiArray`)
  - Order: `[A0, A1, A2, A3]`

### Services

- **`/set_output`** (`std_srvs/SetBool`)
  - Uses ROS parameter `~output_pin` to select which pin to control
- **`/set_period`** (`std_srvs/Trigger`)
  - Uses ROS parameter `~period_ms`

---

## Parameters

| Name | Type | Default | Description |
|----|----|----|----|
| `~port` | string | `/dev/ttyACM0` | Serial device |
| `~baudrate` | int | `115200` | Serial baudrate |
| `~output_pin` | int | `8` | Digital output pin |
| `~period_ms` | int | `50` | ADC publish period (ms) |

---

## Usage

### 1. Upload Arduino firmware

Upload the firmware located at:

```
firmware/arduino_serial.ino
```

using the Arduino IDE.

> Make sure no process is using `/dev/ttyACM0` while uploading.

---

### 2. Build the ROS workspace

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

### 3. Launch the node

```bash
roslaunch arduino_serial arduino_serial.launch
```

---

### 4. Control digital outputs

```bash
rosparam set /arduino_serial_node/output_pin 8
rosservice call /set_output "{data: true}"
```

```bash
rosservice call /set_output "{data: false}"
```

---

### 5. Change ADC period

```bash
rosparam set /arduino_serial_node/period_ms 100
rosservice call /set_period
```

---

## Notes on Permissions

The serial device must be accessible by the user running ROS:

```bash
sudo usermod -a -G dialout $USER
reboot
```

Avoid using:
```bash
chmod 666 /dev/ttyACM0
```
as it is insecure.

---

## Design Philosophy

- Arduino handles **low-level, real-time I/O**
- ROS handles **supervision, configuration, and integration**
- Serial protocol kept:
  - Explicit
  - Human-readable
  - Easy to debug

This architecture is suitable for:
- Mobile robots
- Underwater robots
- Industrial prototypes
- Research platforms

---

## Future Improvements

- Custom ROS messages/services
- ACK/heartbeat from Arduino
- Watchdog for safety
- Migration to C++ or micro-ROS
- systemd integration for autonomous startup

---

## Author

CIRTESU / IRS Lab  
Universitat Jaume I (UJI)
