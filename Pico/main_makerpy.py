from machine import Pin, UART, I2C, SPI, ADC, Timer, PWM
import os
import sdcard
from ssd1306 import SSD1306_I2C
from neopixel import NeoPixel
import time
import utime
from pushbutton import PushButton

# ESP01 on UART0, GP16 and GP17
uart0 = UART(0, baudrate=115200, tx=Pin(16), rx=Pin(17))
# LED on Pico board
ledpin = Pin(25, Pin.OUT)
# Chip temperature sensor
sensor_temp = ADC(4)
conversion_factor = 3.3 / (65535)
# Pico: ADC3 used for VSYS/3
sensor_vsys3 = ADC(Pin(29))
# Pushbuttons
pin_gp20 = Pin(20, Pin.IN, Pin.PULL_UP)
pin_gp21 = Pin(21, Pin.IN, Pin.PULL_UP)
pin_gp22 = Pin(22, Pin.IN, Pin.PULL_UP)
button_gp20 = PushButton(0, 5, pin_gp20)
button_gp21 = PushButton(0, 5, pin_gp21)
button_gp22 = PushButton(0, 5, pin_gp22)
# Timer to refresh button states
button_timer = Timer()

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
#sd = sdcard.SDCard(sd_spi, sd_cs)

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

def button_refresh(timer):
    button_gp20.update(pin_gp20.value())
    button_gp21.update(button_gp21.pin.value())
    button_gp22.update(button_gp22.pin.value())

button_timer.init(mode=Timer.PERIODIC, period=20, callback=button_refresh)

# Logfile for temperatures
file = open("temps.txt", "w")

while True:
    ledpin.toggle()
    read_adctemp = sensor_temp.read_u16() * conversion_factor
    adc_temperature = 27 - (read_adctemp - 0.706)/0.001721
    battery_voltage = 3 * 3.3 * sensor_vsys3.read_u16() / 65535
    print(battery_voltage)

    if button_gp20.checkPushed() is True:
        print("GP20 pressed")
    file.write(str(adc_temperature) + "\n")
    file.flush()
    time.sleep_ms(500)
