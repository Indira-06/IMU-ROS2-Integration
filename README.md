# IMU ROS2 Integration — BNO085 + ROS2 Humble + Python Node

This guide documents the full process of setting up and running a **SparkFun BNO085 IMU sensor** with **ROS2 Humble** using a custom Python node on **WSL2** (Ubuntu 22.04). It includes working steps, what failed, reflections, and how to re-run from scratch.

## Objective

Publish real-time BNO085 IMU data via serial over a ROS2 topic using a custom Python node.

---

## Setup

| Item                                            |
|-------------------------------------------------|
| WSL2 with Ubuntu 22.04                          |
| ROS2 Humble installed in WSL                    |
| Python (3.10+) in WSL                           |
| VSCode on windows(with WSL integration)         |
| SparkFun BNO085 IMU Sensor(I2C mode)            |
| ESP32 microcontroller                           |

---

## Wiring (ESP32 ↔ BNO085 via I2C)

| ESP32 Pin | BNO085 Pin | Function                  |
|-----------|------------|---------------------------|
| 3V3       | VIN        | Power supply (3.3V)       |
| GND       | GND        | Ground                    |
| GPIO19    | SDA        | I2C data line             |
| GPIO18    | SCL        | I2C clock line            |
| GPIO21    | INT        | Interrupt (optional)      | 
| GPIO22    | RST        | Reset sensor (optional)   |

---

## Notes

**SparkFun Defaults**
- **P0** is pulled HIGH internally→ I2C mode.
- **P1** is left LOW or floating → sets default I2C address.
- Do not connect P0 and P1 externally as they are already connected internally (If connected, in my case the IMU was not detected).

**I2C Address**
- IMU was found at address 0x4A in my case. 
- In general IMU uses default address 0x4B. 

---

## Arduino Quick Check (Raw Readings)

**Upload Example**
- File > Examples > SparkFun BNO08x > Raw_Readings

**Slight Modifications** 

Since we are using ESP32 microcontroller instead of Arduino and not using the defalut SDA,SCL pins: 

- **Modify** the below code

```cpp
#include <Wire.h>

#define BNO08X_INT A4
#define BNO08X_RST A5

void setup() {
  Wire.begin();  // Default pins are SDA = GPIO21, SCL = GPIO22
  Serial.begin(115200);
  // BNO085 initialization code here
}

void loop() {
  // Reads and prints sensor data
}
```

- **with** the following code

```cpp
#include <Wire.h>

#define BNO08X_INT 21
#define BNO08X_RST 22

void setup() {
  Wire.begin(19,18);
  Serial.begin(115200);
  // BNO085 initialization code here
}

void loop() {
  // Reads and prints sensor data
}
```
Check serial monitor for IMU data.

## ROS2 Workspace Setup

```bash
# Create and build workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```
## Installing Required Python Packages

```bash
pip install pyserial
```
**VSCode Extensions to Install**

- ms-iot.vscode-ros (ROS2 support)
- ms-python.python (Python)
- ms-vscode.cpptools (C++)
- ms-vscode.remote-wsl (WSL bridge)

Make sure bottom-left of VSCode says:
- WSL:Ubuntu -> Now we're coding inside WSL

## Creating Custom ROS2 Python Package

```bash
cd ~/ros2_ws/src

#writing a ros2 python node to read from serial
ros2 pkg create --build-type ament_python --dependencies rclpy std_msgs serial imu_serial_reader
```

**Writing ```serial_reader_node.py```**
```bash
cd ~/ros2_ws/src/imu_serial_reader/imu_serial_reader
touch serial_reader_node.py
chmod +x serial_reader_node.py
```

Paste this example:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class IMUSerialReader(Node):
    def __init__(self):
        super().__init__('imu_serial_reader')
        self.publisher_ = self.create_publisher(String, 'imu_raw', 10)
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.timer = self.create_timer(0.1, self.read_serial)

    def read_serial(self):
        if self.ser.in_waiting:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                msg = String()
                msg.data = line
                self.publisher_.publish(msg)
                self.get_logger().info(f'Publishing: {line}')
            except Exception as e:
                self.get_logger().error(f"Error reading serial: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = IMUSerialReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Update ```setup.py```**

Edit the ```entry_points``` section:

```python
entry_points={
    'console_scripts': [
        'serial_reader_node = imu_serial_reader.serial_reader_node:main',
    ],
},
```

Then building the workspace:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Serial Port Sharing: Windows -> WSL2

Necessary to bridge Windows USB ports into WSL

**On Windows Powershell as Admin** 

(Right Click on Windows PowerShell and select Run as Adminstrator)

1. **Install usbipd-win (on Windows)**

```powershell
winget install usbipd
```
2. **Bind USB Device**

```powershell
usbipd list
```
Should see something like:

```bash
Silicon Labs CP210x USB to UART Bridge (COM3) at 2-2
```
To Bind:

```poweshell
usbipd bind --busid 2-2
usbipd attach --busid 2-2 --wsl
```

**Install USB tools in WSL2:**

```bash
sudo apt update
sudo apt install linux-tools-generic hwdata usbutils

lsusb
ls /dev/ttyUSB* #look for device path
```

Should detect:
```bash
/dev/ttyUSB0
```
or something similar

Update this path in ```serial_reader_node.py```:

```python
self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
```

**Running the ROS2 Node**
```bash
ros2 run imu_serial_reader serial_reader_node
```
## Running the node (from Scratch)
Whenever system is restarted or WSL/VS Code is closed:

```bash
# 1. Share USB again from Windows Powershell as Administrator
usbipd attach --busid 2-2 --wsl

# 2. Re-source and run ROS2 node
cd ~/ros2_ws
source install/setup.bash
ros2 run imu_serial_reader serial_reader_node
```

In a new terminal (optional):
```bash
#view topic messages
ros2 topic echo /imu_data
```


## Directory structure of Project:
```arduino
ros2_ws/
└── src/
    └── imu_serial_reader/
        ├── imu_serial_reader/
        │   ├── __init__.py
        │   └── serial_reader_node.py
        ├── package.xml
        └── setup.py
```


## Troubleshooting
| Issue                | Solution                                                |
|----------------------|---------------------------------------------------------|
| ```SerialException```|Check ```ls /dev/ttyUSB*```, update path in Python       |
| Data not readable    |Confirm baud rate, try ```115200``` or ```9600```        |
| Port not showing     |Re-bind in Windows using usbipd commands                 |

## Current Status

- ✔ Custom Python node built and runs inside WSL2
- ✔ Reads IMU data over serial and publishes to /imu/data_raw topic
- ✔ USB device shared from Windows → WSL2 using usbipd
- ❌ IMU reports reset often or missing accelerometer output
  - Occasional “Could not enable accelerometer” errors
  - No data showing up in ROS2 node at times
  - Likely hardware or I2C noise issues

But getting even partial data -> topic is a solid milestone!

## Debug Summary

**What I've checked:**
- Power and ground wiring
- If SDA/SCL wires are shorted
- Correct I2C address selected
- Serial Port shared correctly

**What might be wrong**
- WSL2 USB passthrough is unstable
- I2C pullups may be weak
- I2C frequency too high
- Firmware reset loops

**What I might try next**
- Move entire setup to a Linux native system
- Lower I2C from 400kHz to 100kHz
- Maybe add stronger pull-ups on SDA/SCL
- Better shielding in circuit

## Future Scope

The Biped in our lab is currently able to do Squats. We need to get it walking

**Example System Architecture for implementing a walking Biped**

| Device          | Role                                                                                  |
|-----------------|---------------------------------------------------------------------------------------|
| ESP32           |Reads data from 3× BNO085 IMUs over I2C                                                |
| ROS2 PC         |Receives IMU data over serial and publishes: /imu/torso, /imu/left_leg, /imu/right_leg |
| Controller Node      |Subscribes to IMU topics and controls gait                                        |

**Using RViz**
- Add RViz plugin to visualize IMU pose/orientation
- Fuse IMU data with robot_localization or imu_filter_madgwick


**Future Improvements in short**
- Use RViz to visualize orientation.
- Extend node to parse quaternion data.
- Publish to /imu/torso, /imu/left_leg, etc.
- Add timestamp sync between ESP32 and ROS2.

## Final Thoughts
- WSL2 + Hardware is functional but finicky
- IMU reset issues might be due to virtualized USB stack
- Consider full hardware Linux test to isolate software-side bugs

© 2025 Indira C Reddy
