from machine import Pin, I2C, SPI, ADC, Timer, PWM
from utime import sleep
import sys
import select
import os
import sdcard
from pcf8523 import PCF8523

#Board led
Led_Brd = Pin("LED", Pin.OUT)

#I2C0: PCF8523 RTC
i2c0 = I2C(0, sda=Pin(4), scl=Pin(5), freq=400000)
devices = i2c0.scan()
if len(devices) == 0:
  print("No i2c device !")
else:
  print('i2c devices found:', devices)
#RTC init
rtc = PCF8523(i2c0)

#ADC
#adc ports
adc26 = ADC(26)
adc27 = ADC(27)
adc28 = ADC(28)
# Chip temperature sensor
sensor_temp = ADC(4)
conversion_factor = 3.3 / (65535)
# Pico: ADC3 used for VSYS/3
adc_bat = ADC(Pin(29))

#SD CARD
# SD card
sd_cs = Pin(17, Pin.OUT, value = 1)
sd_spi = SPI(0,
             baudrate = 1000000,
             polarity = 0,
             phase = 0,
             bits = 8,
             firstbit = SPI.MSB,
             sck = Pin(18),
             mosi = Pin(19),
             miso = Pin(16))

try:
    # Initialize SD card
    vosd = sdcard.SDCard(sd_spi, sd_cs)
except OSError:
    print("No SD cardid")

print("LED starts flashing...")
while True:
    try:
        Led_Brd.toggle()
        print(rtc.PrintTime())
        sleep(1) # sleep 1sec
    except KeyboardInterrupt:
        break
Led_Brd.off()
print("Finished.")
