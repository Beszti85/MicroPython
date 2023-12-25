from machine import Pin, UART, I2C, SPI, ADC, Timer, PWM
import os
import sdcard
from neopixel import NeoPixel
import time
import utime

# ESP01 on UART0, GP16 and GP17
uart0 = UART(0, baudrate=115200, tx=Pin(16), rx=Pin(17))
# LED on Pico board
ledpin = Pin(25, Pin.OUT)
# Chip temperature sensor
sensor_temp = ADC(4)
conversion_factor = 3.3 / (65535)
# Pushbuttons
button_gp20 = Pin(20, Pin.IN)
button_gp21 = Pin(21, Pin.IN)
button_gp22 = Pin(22, Pin.IN)
# Neopixel led: GP28
pin_NP = Pin(28, Pin.OUT)
led_np = NeoPixel(pin_NP, 1)
# ADC inputs: GP26 and GP27
adc_0 = ADC(Pin(26))
adc_1 = ADC(Pin(27))
# I2C OLED Display
i2c=I2C(1,sda=Pin(6), scl=Pin(7), freq=400000)
# Timer for neopixel led
np_timer = Timer()
np_cycle = 0
# PWM channels for audio jack
audio_left  = PWM(Pin(18))
audio_right = PWM(Pin(19))
# SD card
sd_cs = Pin(15, Pin.OUT)
sd_spi = SPI(1,
             baudrate = 5000000,
             polarity = 0,
             phase = 0,
             bits = 8,
             firstbit = SPI.MSB,
             sck = Pin(10),
             mosi = Pin(11),
             miso = Pin(8))

# Initialize SD card
sd = sdcard.SDCard(sd_spi, sd_cs)

# Short delay to stop I2C falling over
time.sleep(1)

def np_change(timer):
    global np_cycle
    if np_cycle == 0:
        led_np[0] = (63, 0, 0)
        np_cycle = 1
    elif np_cycle == 1:
        led_np[0] = (0, 63, 0)
        np_cycle = 2
    else:
        led_np[0] = (0, 0, 63)
        np_cycle = 0
    led_np.write()

np_timer.init(mode=Timer.PERIODIC, period=2000, callback=np_change)

while True:
    ledpin.toggle()
    read_adctemp = sensor_temp.read_u16() * conversion_factor
    adc_temperature = 27 - (read_adctemp - 0.706)/0.001721
    time.sleep_ms(500)
