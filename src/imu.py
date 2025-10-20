#!/usr/bin/env python3
import rospy
import serial
import time
import re
import math

from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_slerp, quaternion_multiply, quaternion_conjugate
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped 

class RP2040IMU:
	def __init__(self):
		rospy.init_node('rp2040_imu_node')

		self.param_serial_port = rospy.get_param('~port', "/dev/ttyACM0")
		self.param_baud_rate = rospy.get_param('~baud_rate', "115200")

		self.connect_to_serial_port()

		self.imu_pub = rospy.Publisher("/rp2040_imu/data_raw", Imu, queue_size=10)

		rospy.loginfo("IMU Ready")


	def connect_to_serial_port(self):
		while True:
			try:
				self.port = serial.Serial(port=self.param_serial_port, baudrate=self.param_baud_rate)
				print("Connected to serial port.")
				break  # Exit the loop if the connection is successful
			except serial.SerialException as e:
				print(f"Failed to connect to serial port: {e}")
				print("Retrying in 2 seconds...")
				time.sleep(2)  # Wait for 2 seconds before retrying

	def publish_imu_msg(self, ax, ay, az, gx, gy, gz):

		imu_msg = Imu()
		imu_msg.header.stamp = rospy.Time.now()
		imu_msg.header.frame_id = "rp2040_imu_link"

		# Orientation (quaternion)
		imu_msg.orientation.x = 0
		imu_msg.orientation.y = 0
		imu_msg.orientation.z = 0
		imu_msg.orientation.w = 0
		imu_msg.orientation_covariance = [
			9999, 0, 0,
			0, 9999, 0,
			0, 0, 9999
		]

		# Angular velocity, deg/s to rad/s
		imu_msg.angular_velocity.x = math.radians(gx)
		imu_msg.angular_velocity.y = math.radians(gy)
		imu_msg.angular_velocity.z = math.radians(gz)
		imu_msg.angular_velocity_covariance = [
			0.0001, 0, 0,
			0, 0.0001, 0,
			0, 0, 0.0001
		]

		# Linear acceleration, g to m/s²
		imu_msg.linear_acceleration.x = ax * 9.80665
		imu_msg.linear_acceleration.y = ay * 9.80665
		imu_msg.linear_acceleration.z = az * 9.80665
		imu_msg.linear_acceleration_covariance = [
			0.04, 0, 0,
			0, 0.04, 0,
			0, 0, 0.04
		]
		self.imu_pub.publish(imu_msg)

	def receive_data(self):
		try:
			message = self.port.readline().decode("utf-8","ignore").strip()
			if message.startswith('#') and message.endswith('#'):
				split = message.replace("#","").split(",")
				if len(split) == 6:
					self.publish_imu_msg(
						float(split[0]), #raw accel x
						float(split[1]), #raw accel y
						float(split[2]),  #raw accel z

						float(split[3]), #raw gyro x
						float(split[4]), #raw gyro y
						float(split[5]) #raw gyro z
					)
		except Exception as e:
			print(f"Connection lost: {e}")
			print("Attempting to reconnect...")
			self.port.close()  # Close the port if it is open
			self.connect_to_serial_port()  # Attempt to reconnect



	def cleanup(self):
		self.port.close()

imu = RP2040IMU()
rospy.on_shutdown(imu.cleanup)

while not rospy.is_shutdown():
	imu.receive_data()