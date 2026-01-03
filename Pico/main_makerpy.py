from machine import Pin, UART, I2C, SPI, ADC, Timer, PWM
import os
import sdcard
from ssd1306 import SSD1306_I2C
from neopixel import NeoPixel
from ds1307 import DS1307
import time
import utime
import uos
from pushbutton import PushButton
import dht
import bme280
from hcsr04 import HCSR04

def displ_test(disp):
    disp.fill(0)
    disp.fill_rect(0, 0, 32, 32, 1)
    disp.fill_rect(2, 2, 28, 28, 0)
    disp.vline(9, 8, 22, 1)
    disp.vline(16, 2, 22, 1)
    disp.vline(23, 8, 22, 1)
    disp.fill_rect(26, 24, 2, 4, 1)
    disp.text('MicroPython', 40, 0, 1)
    disp.text('SSD1306', 40, 12, 1)
    disp.text('OLED 128x64', 40, 24, 1)
    disp.text('Next line code', 0, 36, 2)
    disp.show()

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
button_gp20 = PushButton(0, 5, Pin(20, Pin.IN))
button_gp21 = PushButton(0, 5, Pin(21, Pin.IN))
button_gp22 = PushButton(0, 5, Pin(22, Pin.IN))
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
devices = i2c.scan()
if len(devices) == 0:
  print("No i2c device !")
else:
  print('i2c devices found:', devices)
display = SSD1306_I2C(128, 64, i2c)
displ_test(display)
# DS1307 RTC
rtc = DS1307(i2c)
#BME280
bme = bme280.BME280(i2c=i2c)
# Timer for neopixel led
np_timer = Timer()
np_cycle = 0
# PWM channels for audio jack
audio_left  = PWM(Pin(18))
audio_right = PWM(Pin(19))
# SD card
sd_cs = Pin(15, Pin.OUT, value = 1)
sd_spi = SPI(1,
             baudrate = 1000000,
             polarity = 0,
             phase = 0,
             bits = 8,
             firstbit = SPI.MSB,
             sck = Pin(10),
             mosi = Pin(11),
             miso = Pin(12))

# Initialize SD card
sd = sdcard.SDCard(sd_spi, sd_cs)

# Mount filesystem
vfs = uos.VfsFat(sd)
uos.mount(vfs, "/sd")

# Create a file and write something to it
with open("/sd/pico.txt", "w") as file:
    file.write("1. Hello, world!\r\n")

sensor_us = HCSR04(trigger_pin=2, echo_pin=3, echo_timeout_us=30000)

#Servo motor PWM test: Pin0
pwm_servo = PWM(Pin(0))
pwm_servo.freq(50)

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
    button_gp20.update(button_gp20.pin.value())
    button_gp21.update(button_gp21.pin.value())
    button_gp22.update(button_gp22.pin.value())

button_timer.init(mode=Timer.PERIODIC, period=20, callback=button_refresh)

# Logfile for temperatures
#file = open("temps.txt", "w")
file = open("/sd/pico.txt", "w")

pwm_pulse = 0
sqw_val   = 0

while True:
    ledpin.toggle()
    read_adctemp = sensor_temp.read_u16() * conversion_factor
    adc_temperature = 27 - (read_adctemp - 0.706)/0.001721
    battery_voltage = 3 * 3.3 * sensor_vsys3.read_u16() / 65535
    distance = sensor_us.distance_cm()
    print('Distance:', distance, 'cm')
    print(f"Battery voltage: {battery_voltage}V")
    print(f"Lightning value: {adc_1.read_u16()}")
    sensor_dht11.measure()
    dht11_temp = (sensor_dht11.temperature())
    dht11_hum = (sensor_dht11.humidity())
    print("Internal tempreature: {}C".format(adc_temperature))
    print("Temperature: {}C".format(dht11_temp))
    print("Humidity: {}%".format(dht11_hum))

    if button_gp20.checkPushed() is True:
        pwm_pulse += 10
        print("GP20 pressed")
    if button_gp21.checkPushed() is True:
        pwm_pulse -= 10
        rtc.SquareWave(0, 0)
        print("GP21 pressed")
    if button_gp22.checkPushed() is True:
        rtc.SquareWave(1, sqw_val)
        if sqw_val < 3:
            sqw_val += 1
        else:
            sqw_val = 0
        print("GP22 pressed")
    print(pwm_pulse)
    print(bme.values)
    print(rtc.PrintTime())
    pwm_servo.duty_ns(pwm_pulse * 1000)
    # Write variables into SD card
    file.write(f"------------------------------------------------\n")
    file.write(f"Internal temperature: {adc_temperature}C\n")
    file.write(f"Battery voltage: {battery_voltage}V\n")
    file.write(f"Temperature: {dht11_temp}C\n")
    file.write(f"Humidity: {dht11_hum}%\n")
    file.write(f"Lightning value: {adc_1.read_u16()}\n")

    file.flush()
    time.sleep_ms(500)
