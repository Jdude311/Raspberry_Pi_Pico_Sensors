# To view the pico's STDOUT: screen /dev/ttyACM0

from board import Pin
import utime

led = Pin(28, Pin.OUT)
pir = Pin(16, Pin.IN, Pin.PULL_UP)
led.high()
