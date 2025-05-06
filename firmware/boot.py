import time
import sys
import uselect
import utime

from machine import Pin, Timer

def is_repl_active(timeout_ms=10000):
    poller = uselect.poll()
    poller.register(sys.stdin, uselect.POLLIN)
    res = poller.poll(timeout_ms)  # Wait for up to timeout_ms
    poller.unregister(sys.stdin)
    
    if len(res) > 0:
        char = sys.stdin.read(1)
        return char == 'a'
    return False

led = Pin(6, Pin.OUT)
led.value(1)

if is_repl_active():
	led.value(1)
else:
	led.value(0)
	
	from uart import PacketSerial
	serial = PacketSerial()



