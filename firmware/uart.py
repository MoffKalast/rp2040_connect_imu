import time
import utime
import struct
import math

from machine import UART, Timer, Pin
from lib_imu import Imu
from temperature import Temperature

class PacketSerial:
	def __init__(self):
		self.imu = Imu()
		self.temp = Temperature()		
		self.temp_prescaler = 0;
		
		self.timer = Timer(period=10, mode=Timer.PERIODIC, callback=self.send_imu_data)
		
	def send_imu_data(self, timer):
		#100 hz
		print(self.imu.read())
		
		self.temp_prescaler +=1
		
		#1 hz
		if self.temp_prescaler >= 100:
			self.temp_prescaler = 0
			print(self.temp.read())

	def cleanup(self):
		self.timer.deinit()
	