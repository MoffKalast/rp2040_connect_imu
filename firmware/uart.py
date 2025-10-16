import time
import utime
import struct
import math

from machine import UART, Timer, Pin
from lib_imu import Imu

class PacketSerial:
	def __init__(self):
		self.imu = Imu()
		self.imu_timer = Timer(period=10, mode=Timer.PERIODIC, callback=self.send_imu_data)
		
	def send_imu_data(self, timer):
		#100 hz
		print(self.imu.read())

	def cleanup(self):
		self.imu_timer.deinit()
	