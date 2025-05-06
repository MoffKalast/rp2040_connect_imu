# RP2040 Connect IMU ROS

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

Micropython firmware and ROS Noetic driver for the RP2040 Connect board with an onboard IMU. It does onboard fusion with arbitrary precision floats for high numerical stability.

## Setup

Flash the micropython firmware to the RP2040 and upload the files in /firmware.

Create udev rule `/etc/udev/rules.d/99-ttyIMU.rules`:
```bash
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="025e", SYMLINK+="ttyIMU"
```

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Params

Example launch:
```xml
<node name="rp2040_imu_node" pkg="rp2040_connect_imu" type="imu.py" output="screen">
	<param name="port" value="/dev/ttyIMU" />
	<param name="baud_rate" value="115200" />
</node>
```

## Published Topics

- `/rp2040_imu/data` (Imu), quaternion orientation and raw gyro and accel data

- `/rp2040_imu/pose` (PoseStamped), debug pose publisher
