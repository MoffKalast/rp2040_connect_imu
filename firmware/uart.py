import time
import utime
import struct
import math

from machine import UART, Timer, Pin
from lib_imu_quat import Imu

class PacketSerial:
	def __init__(self):
		self.imu = Imu()
		self.imu_timer = Timer(period=10, mode=Timer.PERIODIC, callback=self.send_imu_data)
		#self.sending_prescaler = 0
		
	def send_imu_data(self, timer):
		self.imu.update()
	
		#50 hz
		#if self.sending_prescaler == 0:
		print(self.imu.get_printout())
	
		#self.sending_prescaler = (self.sending_prescaler  + 1) % 4

	def cleanup(self):
		self.imu_timer.deinit()
	
