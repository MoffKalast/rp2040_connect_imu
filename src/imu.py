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

		self.imu_pub = rospy.Publisher("/rp2040_imu/data", Imu, queue_size=10)
		self.pose_pub = rospy.Publisher("/rp2040_imu/pose", PoseStamped, queue_size=10)

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

	def publish_imu_msg(self, x, y, z, w, gx, gy, gz, ax, ay, az):

		imu_msg = Imu()
		imu_msg.header.stamp = rospy.Time.now()
		imu_msg.header.frame_id = "imu_link"

		# Orientation (quaternion)
		imu_msg.orientation.x = x
		imu_msg.orientation.y = y
		imu_msg.orientation.z = z
		imu_msg.orientation.w = w
		imu_msg.orientation_covariance = [
			0.0025, 0, 0,
			0, 0.0025, 0,
			0, 0, 0.0025
		]

		# Angular velocity (rad/s)
		imu_msg.angular_velocity.x = gx
		imu_msg.angular_velocity.y = gy
		imu_msg.angular_velocity.z = gz
		imu_msg.angular_velocity_covariance = [
			0.0001, 0, 0,
			0, 0.0001, 0,
			0, 0, 0.0001
		]

		# Linear acceleration (m/s²)
		imu_msg.linear_acceleration.x = ax
		imu_msg.linear_acceleration.y = ay
		imu_msg.linear_acceleration.z = az
		imu_msg.linear_acceleration_covariance = [
			0.04, 0, 0,
			0, 0.04, 0,
			0, 0, 0.04
		]
		self.imu_pub.publish(imu_msg)

		pose_msg = PoseStamped()
		pose_msg.header.stamp = rospy.Time.now()
		pose_msg.header.frame_id = "imu_link"
		pose_msg.pose.position.x = 0
		pose_msg.pose.position.y = 0
		pose_msg.pose.position.z = 0
		pose_msg.pose.orientation.x = x
		pose_msg.pose.orientation.y = y
		pose_msg.pose.orientation.z = z
		pose_msg.pose.orientation.w = w
		self.pose_pub.publish(pose_msg)

	def receive_data(self):
		while not rospy.is_shutdown():
			try:
				message = self.port.readline().decode("utf-8","ignore").strip()
				if message.startswith('#') and message.endswith('#'):
					split = message.replace("#","").split(",")
					if len(split) == 10:
						self.publish_imu_msg(
							float(split[0]), #x
							float(split[1]), #y
							float(split[2]), #z
							float(split[3]), #w

							float(split[4]), #raw gyro x
							float(split[5]), #raw gyro y
							float(split[6]), #raw gyro z

							float(split[7]), #raw accel x
							float(split[8]), #raw accel y
							float(split[9])  #raw accel z
						)
			except (serial.SerialException, OSError) as e:
				print(f"Connection lost: {e}")
				print("Attempting to reconnect...")
				self.port.close()  # Close the port if it is open
				self.connect_to_serial_port()  # Attempt to reconnect



	def cleanup(self):
		self.port.close()

imu = RP2040IMU()

rate = rospy.Rate(rospy.get_param('rate', 60))
rospy.on_shutdown(imu.cleanup)

while not rospy.is_shutdown():
	imu.receive_data()
	rate.sleep()