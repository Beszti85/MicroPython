from machine import Pin, UART, I2C, SPI, ADC, Timer, PWM
import os
#import sdcard
#from ssd1306 import SSD1306_I2C
#from neopixel import NeoPixel
#from ds1307 import DS1307
import time
import utime
import _thread
from hcsr04 import HCSR04

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
# buzzer
buzzer = PWM(Pin(22))
buzzer.freq(1000)
# motor driver pins
motor1_plus = PWM(Pin(8))
motor1_minus = Pin(9, Pin.OUT)
motor1_minus.value(0)
#motor2_plus = PWM(Pin(10))
#motor2_minus = PWM(Pin(11))

sensor_us = HCSR04(trigger_pin=2, echo_pin=3, echo_timeout_us=30000)

max_duty = 7864 
min_duty = 1802
half_duty = int(max_duty/2)

servo15.freq(50)
#read battery voltage
print(2 * 3.3 * adc_bat.read_u16() / 65535)

motor1_plus.freq(10000)
#motor1_minus.duty_u16(0)
#duty_motor = 15000

while(1):
    distance = sensor_us.distance_cm()
    duty_motor = int(distance * 65535 / 400)
    motor1_plus.duty_u16(duty_motor)
    print('Distance:', distance, 'cm')
    led_dbg.value(1)
    servo15.duty_u16(min_duty)
    buzzer.duty_u16(min_duty)
    #duty_motor += 3000
    if duty_motor > 65000:
        duty_motor = 0
    time.sleep(1)
    motor1_plus.duty_u16(duty_motor)
    led_dbg.value(0)
    servo15.duty_u16(half_duty)
    buzzer.duty_u16(max_duty)
    time.sleep(1)
    servo15.duty_u16(max_duty)
    buzzer.duty_u16(0)
    time.sleep(1)
