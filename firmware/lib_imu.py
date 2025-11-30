import time
import math

from lsm6dsox import LSM6DSOX
from machine import Pin, I2C

def median(values):
    sorted_values = sorted(values)
    mid = len(sorted_values) // 2
    if len(sorted_values) % 2 == 0:
        return (sorted_values[mid - 1] + sorted_values[mid]) / 2
    return sorted_values[mid]

class Imu:
	def __init__(self, mult_gyro_x=1.0, mult_gyro_y=1.0, mult_gyro_z=1.0):	
		self.lsm = LSM6DSOX(I2C(0, scl=Pin(13), sda=Pin(12)))
		
		self.mult_gyro_x = mult_gyro_x
		self.mult_gyro_y = mult_gyro_y
		self.mult_gyro_z = mult_gyro_z
		self.calibrate()


	def calibrate(self, samples=50, delay_ms=10):
		gyro_x, gyro_y, gyro_z = [], [], []

		for _ in range(samples):
			ax, ay, az = self.lsm.accel()
			gx, gy, gz = self.lsm.gyro()
			
			gyro_x.append(gx)
			gyro_y.append(gy)
			gyro_z.append(gz)
			
			time.sleep_ms(delay_ms)
			
		self.calib_gyro_x = median(gyro_x)
		self.calib_gyro_y = median(gyro_y)
		self.calib_gyro_z = median(gyro_z)

	def read(self):
		accel_x, accel_y, accel_z = self.lsm.accel()
		gyro_x, gyro_y, gyro_z = self.lsm.gyro()
		
		gx = gyro_x - self.calib_gyro_x
		gy = gyro_y - self.calib_gyro_y
		gz = gyro_z - self.calib_gyro_z
		
		return f"#{accel_x},{accel_y},{accel_z},{gx},{gy},{gz}#"

