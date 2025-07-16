from machine import Pin, UART, I2C, SPI, ADC, Timer, PWM
import os
#import sdcard
#from ssd1306 import SSD1306_I2C
#from neopixel import NeoPixel
#from ds1307 import DS1307
import time
import utime
import _thread

led_dbg = Pin(16, Pin.OUT)
#servo pins
servo12 = PWM(Pin(12))
servo13 = PWM(Pin(13))
servo14 = PWM(Pin(14))
servo15 = PWM(Pin(15))
#push buttons
button20 = Pin(20, Pin.IN)
#adc ports
adc26 = ADC(26)
adc27 = ADC(27)
adc28 = ADC(28)
adc_bat = ADC(29)

max_duty = 7864
min_duty = 1802
half_duty = int(max_duty/2)

servo15.freq(50)
#read battery voltage
print(2 * 3.3 * adc_bat.read_u16() / 65535)

while(1):
    led_dbg.value(1)
    servo15.duty_u16(min_duty)
    time.sleep(1)
    led_dbg.value(0)
    servo15.duty_u16(half_duty)
    time.sleep(1)
    servo15.duty_u16(max_duty)
    time.sleep(1)
