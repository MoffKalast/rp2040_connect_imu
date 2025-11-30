# RP2040 Connect IMU ROS

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

Micropython firmware and ROS Noetic driver for the RP2040 Connect board with an onboard IMU. 

This branch only reports raw accel and gyro values at 100hz, micropython firmware v1.25.0 (2025-04-15) is required.

## Setup

Flash the micropython firmware to the RP2040 and upload the files in /firmware.

Create udev rule `/etc/udev/rules.d/99-ttyIMU.rules`:
```bash
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="025e", SYMLINK+="ttyIMU", MODE="0666"
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
	<param name="accel_correction_gain" value="0.03" />
	<param name="gyro_scale" value="1.15" />
</node>
```

## Published Topics

- `/rp2040_imu/data_raw` (Imu), raw gyro and accel data, (100 Hz)

- `/rp2040_imu/data` (Imu),  fused quaternion from gyro and accel data (33 Hz)
 
- `/rp2040_imu/temperature` (Temperature), RP2040 built in temperature sensor (1 Hz)
