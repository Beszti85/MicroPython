from machine import Pin, UART, I2C, SPI, ADC, Timer, PWM
import os
#import sdcard
#from ssd1306 import SSD1306_I2C
#from neopixel import NeoPixel
#from ds1307 import DS1307
import time
import utime

led_dbg = Pin(16, Pin.OUT)
servo15 = PWM(Pin(15))

max_duty = 7864
min_duty = 1802
half_duty = int(max_duty/2)

servo15.freq(50)

while(1):
    led_dbg.value(1)
    servo15.duty_u16(min_duty)
    time.sleep(1)
    led_dbg.value(0)
    servo15.duty_u16(half_duty)
    time.sleep(1)
    servo15.duty_u16(max_duty)
    time.sleep(1)