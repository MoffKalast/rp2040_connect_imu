#!/usr/bin/env python3
import rospy
import serial
import time
import math
import numpy as np

from tf.transformations import (
    quaternion_from_euler,
    euler_from_quaternion,
    quaternion_slerp,
    quaternion_multiply,
    quaternion_conjugate,
)
from sensor_msgs.msg import Imu


class RP2040IMU:
    def __init__(self):
        rospy.init_node('rp2040_imu_node')

        self.param_serial_port = rospy.get_param('~port', "/dev/ttyACM0")
        self.param_baud_rate = rospy.get_param('~baud_rate', "115200")

        self.connect_to_serial_port()
        self.imu_pub = rospy.Publisher("/rp2040_imu/data_raw", Imu, queue_size=10)

        # Orientation state
        self.q_global = np.array([0.0, 0.0, 0.0, 1.0])  # x, y, z, w
        self.last_time = rospy.Time.now()

        rospy.loginfo("IMU Ready")

    def connect_to_serial_port(self):
        while True:
            try:
                self.port = serial.Serial(port=self.param_serial_port, baudrate=self.param_baud_rate)
                print("Connected to serial port.")
                break
            except serial.SerialException as e:
                print(f"Failed to connect to serial port: {e}")
                print("Retrying in 2 seconds...")
                time.sleep(2)

    def fuse_orientation(self, ax, ay, az, gx, gy, gz):
        """Integrate gyro, correct with accelerometer."""

        # --- Time delta
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        self.last_time = now
        if dt <= 0 or dt > 0.05:  # skip unreasonable intervals
            return self.q_global

        # --- Gyro integration (radians)
        wx, wy, wz = math.radians(gx), math.radians(gy), math.radians(gz)
        omega = np.array([wx, wy, wz])
        omega_norm = np.linalg.norm(omega)

        if omega_norm > 1e-6:
            # Small rotation quaternion from angular velocity
            theta_over_two = 0.5 * omega_norm * dt
            axis = omega / omega_norm
            dq = np.hstack((axis * math.sin(theta_over_two), math.cos(theta_over_two)))
            self.q_global = quaternion_multiply(self.q_global, dq)
            self.q_global /= np.linalg.norm(self.q_global)

        # --- Accel correction
        acc = np.array([ax, ay, az])
        acc_norm = np.linalg.norm(acc)
        if acc_norm < 1e-3:
            return self.q_global  # ignore invalid data
        acc /= acc_norm

        # Compute roll, pitch from accel (gravity vector)
        pitch = math.atan2(-acc[0], math.sqrt(acc[1] ** 2 + acc[2] ** 2))
        roll = math.atan2(acc[1], acc[2])

        # Keep yaw from current orientation
        yaw = euler_from_quaternion(self.q_global)[2]

        q_acc = quaternion_from_euler(roll, pitch, yaw)

        # Fuse using SLERP (small correction)
        q_fused = quaternion_slerp(self.q_global, q_acc, 0.03)
        self.q_global = q_fused / np.linalg.norm(q_fused)

        return self.q_global

    def publish_imu_msg(self, ax, ay, az, gx, gy, gz):
        q = self.fuse_orientation(ax, ay, az, gx, gy, gz)

        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "rp2040_imu_link"

        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]
        imu_msg.orientation_covariance = [
            0.01, 0, 0,
            0, 0.01, 0,
            0, 0, 0.01,
        ]

        imu_msg.angular_velocity.x = math.radians(gx)
        imu_msg.angular_velocity.y = math.radians(gy)
        imu_msg.angular_velocity.z = math.radians(gz)
        imu_msg.angular_velocity_covariance = [
            0.0001, 0, 0,
            0, 0.0001, 0,
            0, 0, 0.0001,
        ]

        imu_msg.linear_acceleration.x = ax * 9.80665
        imu_msg.linear_acceleration.y = ay * 9.80665
        imu_msg.linear_acceleration.z = az * 9.80665
        imu_msg.linear_acceleration_covariance = [
            0.04, 0, 0,
            0, 0.04, 0,
            0, 0, 0.04,
        ]

        self.imu_pub.publish(imu_msg)

    def receive_data(self):
        try:
            message = self.port.readline().decode("utf-8", "ignore").strip()
            if message.startswith('#') and message.endswith('#'):
                split = message.replace("#", "").split(",")
                if len(split) == 6:
                    self.publish_imu_msg(
                        float(split[0]),
                        float(split[1]),
                        float(split[2]),
                        float(split[3]),
                        float(split[4]),
                        float(split[5])
                    )
        except Exception as e:
            print(f"Connection lost: {e}")
            print("Attempting to reconnect...")
            try:
                self.port.close()
            except Exception:
                pass
            self.connect_to_serial_port()

    def cleanup(self):
        self.port.close()


if __name__ == '__main__':
    imu = RP2040IMU()
    rospy.on_shutdown(imu.cleanup)
    while not rospy.is_shutdown():
        imu.receive_data()
 
