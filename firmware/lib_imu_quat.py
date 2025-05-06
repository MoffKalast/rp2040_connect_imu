import time
import math

from mpy_decimal import *

from lsm6dsox import LSM6DSOX
from machine import Pin, I2C

from math import sin, cos, acos, sqrt, atan2, radians

def median(values):
    sorted_values = sorted(values)
    mid = len(sorted_values) // 2
    if len(sorted_values) % 2 == 0:
        return (sorted_values[mid - 1] + sorted_values[mid]) / 2
    return sorted_values[mid]

def get_quaternion_from_euler(roll, pitch, yaw):
	r_half = roll/2
	p_half = pitch/2
	y_half = yaw/2

	roll_sin = sin(r_half)
	roll_cos = cos(r_half)

	pitch_sin = sin(p_half)
	pitch_cos = cos(p_half)

	yaw_sin = sin(y_half)
	yaw_cos = cos(y_half)

	qx = roll_sin * pitch_cos * yaw_cos - roll_cos * pitch_sin * yaw_sin
	qy = roll_cos * pitch_sin * yaw_cos + roll_sin * pitch_cos * yaw_sin
	qz = roll_cos * pitch_cos * yaw_sin - roll_sin * pitch_sin * yaw_cos
	qw = roll_cos * pitch_cos * yaw_cos + roll_sin * pitch_sin * yaw_sin
	return [qx, qy, qz, qw]

def quaternion_to_yaw(w, x, y, z):
    x = float(str(x))
    y = float(str(y))
    z = float(str(z))
    w = float(str(w))
    return atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

class Imu:
	def __init__(self, mult_gyro_x=1.15, mult_gyro_y=1.15, mult_gyro_z=1.15):	
		self.lsm = LSM6DSOX(I2C(0, scl=Pin(13), sda=Pin(12)))
		
		self.mult_gyro_x = mult_gyro_x
		self.mult_gyro_y = mult_gyro_y
		self.mult_gyro_z = mult_gyro_z
		self.reset()

		self.last_update = time.ticks_ms()
		self.slerp_prescaler = 0

	def mult(self, quat):
		
		dw = DecimalNumber(f"{quat[3]:.8f}")
		dx = DecimalNumber(f"{quat[0]:.8f}")
		dy = DecimalNumber(f"{quat[1]:.8f}")
		dz = DecimalNumber(f"{quat[2]:.8f}")

		aw = self.w.clone()
		ax = self.x.clone()
		ay = self.y.clone()
		az = self.z.clone()

		self.w = aw * dw - ax * dx - ay * dy - az * dz
		self.x = aw * dx + ax * dw + ay * dz - az * dy
		self.y = aw * dy - ax * dz + ay * dw + az * dx
		self.z = aw * dz + ax * dy - ay * dx + az * dw

	def slerp(self, q2, t):
		
		x2, y2, z2, w2 = q2
		
		x1 = float(str(self.x))
		y1 = float(str(self.y))
		z1 = float(str(self.z))
		w1 = float(str(self.w))

		dot = x1 * x2 + y1 * y2 + z1 * z2 + w1 * w2
		dot = max(-1.0, min(1.0, dot))

		if dot < 0.0:
			x2 = -x2
			y2 = -y2
			z2 = -z2
			w2 = -w2
			dot = -dot

		if dot > 0.9995:
			scale0 = 1.0 - t
			scale1 = t
		else:
			theta_0 = acos(dot)
			sin_theta_0 = sin(theta_0)
			sin_theta = sin(theta_0 * t)
			
			scale0 = sin(theta_0 * (1.0 - t)) / sin_theta_0
			scale1 = sin_theta / sin_theta_0

		x = scale0 * x1 + scale1 * x2
		y = scale0 * y1 + scale1 * y2
		z = scale0 * z1 + scale1 * z2
		w = scale0 * w1 + scale1 * w2

		self.x = DecimalNumber(f"{x:.8f}")
		self.y = DecimalNumber(f"{y:.8f}")
		self.z = DecimalNumber(f"{z:.8f}")
		self.w = DecimalNumber(f"{w:.8f}")

	def get_printout(self):
		return f"#{self.x},{self.y},{self.z},{self.w},{self.raw_gx},{self.raw_gy},{self.raw_gz},{self.raw_ax},{self.raw_ay},{self.raw_az}#"

	def calibrate(self, samples=50, delay_ms=10):
		accel_x, accel_y, accel_z = [], [], []
		gyro_x, gyro_y, gyro_z = [], [], []

		for _ in range(samples):
			ax, ay, az = self.lsm.accel()
			gx, gy, gz = self.lsm.gyro()
			
			accel_x.append(ax)
			accel_y.append(ay)
			accel_z.append(az)
			gyro_x.append(gx)
			gyro_y.append(gy)
			gyro_z.append(gz)
			
			time.sleep_ms(delay_ms)

		self.calib_accel_x = median(accel_x)
		self.calib_accel_y = median(accel_y)
		self.calib_accel_z = median(accel_z) - 1.0  # Subtract gravity
		self.calib_gyro_x = median(gyro_x)
		self.calib_gyro_y = median(gyro_y)
		self.calib_gyro_z = median(gyro_z)

	def reset(self):		
		self.x = DecimalNumber()
		self.y = DecimalNumber()
		self.z = DecimalNumber()
		self.w = DecimalNumber("1.0")
		self.accel_pitch = 0
		self.accel_roll = 0
		self.accel_roll = 0
		self.calibrate()

	def update(self):
		current_time = time.ticks_ms()
		dt = (current_time - self.last_update) / 1000.0
		self.last_update = current_time

		# Read IMU data
		accel_x, accel_y, accel_z = self.lsm.accel()
		gyro_x, gyro_y, gyro_z = self.lsm.gyro()
		
		self.raw_ax = accel_x
		self.raw_ay = accel_y
		self.raw_az = accel_z
		
		self.raw_gx = gyro_x
		self.raw_gy = gyro_y
		self.raw_gz = gyro_z
		
		#accel_x -= self.calib_accel_x
		#accel_y -= self.calib_accel_y
		#accel_z -= self.calib_accel_z
		
		delta_pitch = (gyro_x - self.calib_gyro_x) * dt * self.mult_gyro_x
		delta_roll = -(gyro_y - self.calib_gyro_y) * dt * self.mult_gyro_y
		delta_yaw = (gyro_z - self.calib_gyro_z) * dt * self.mult_gyro_z
		
		self.mult(get_quaternion_from_euler(
			radians(delta_roll),
			radians(delta_pitch),
			radians(delta_yaw)
		))
		
		# Calculate self.pitch and self.roll from accelerometer data
		raw_accel_pitch = -atan2(-accel_y, sqrt(accel_x**2 + accel_z**2))
		raw_accel_roll = atan2(accel_x, accel_z)
		
		#Low pass filter
		self.accel_pitch = self.accel_pitch * 0.8 + raw_accel_pitch * 0.2
		self.accel_roll = self.accel_roll * 0.8 + raw_accel_roll * 0.2
		
		if self.slerp_prescaler == 0:
			self.accel_yaw = quaternion_to_yaw(self.w, self.x, self.y, self.z)
			accel_quat = get_quaternion_from_euler(self.accel_roll, self.accel_pitch, self.accel_yaw)
			self.slerp(accel_quat, 0.05)
			
		self.slerp_prescaler = (self.slerp_prescaler + 1)%4
		
		
