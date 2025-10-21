from machine import ADC, Pin
import time

class Temperature:
	def __init__(self):
		self.adc = ADC(4)

	def read(self):
		raw = float(self.adc.read_u16()) * 3.3 / 65535
		celsius = 27 - (raw - 0.706)/0.001721
		return f"#{celsius}#"