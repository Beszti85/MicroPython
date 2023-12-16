from machine import Pin, UART, I2C, SPI, ADC
from neopixel import NeoPixel
import time
import utime

# ESP01 on UART0, GP16 and GP17
uart0 = UART(0, baudrate=115200, tx=Pin(16), rx=Pin(17))
# LED on Pico board
ledpin = Pin(25, Pin.OUT)
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

# Short delay to stop I2C falling over
time.sleep(1) 

while True:
    ledpin.toggle()
    time.sleep_ms(500)
