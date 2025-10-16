import time
import sys
import uselect
import utime

from machine import Pin, Timer

led = Pin(6, Pin.OUT)
led.value(1)

#connect a jumper to D2 and GND to enable REPL
debug = Pin(25, Pin.IN, Pin.PULL_UP)
time.sleep(0.5)

if not debug.value():
	led.value(1)
else:
	led.value(0)
	
	from uart import PacketSerial
	serial = PacketSerial()
