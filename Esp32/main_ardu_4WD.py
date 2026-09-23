import machine
from machine import Pin, SPI, I2C, ADC
import config
import network
import time
import bme280
from ds1307 import DS1307
import sdcard

led_board = Pin(2, Pin.OUT)
#i2c bus - BME280 and EEPROM AA24LC256
scl_pin = Pin(22)
sda_pin = Pin(21)
i2c_board = I2C(sda = sda_pin, scl = scl_pin, freq = 200000)

time.sleep(2)

print('Scan i2c bus...')
devices = i2c_board.scan()

if len(devices) == 0:
  print("No i2c device !")
else:
  print('i2c devices found:',len(devices))

#BME280
#I habme = bme280.BME280(i2c=i2c_board)

# RTC: DS1307
rtc = DS1307(i2c_board)

# SD card
sd_cs = Pin(5, Pin.OUT, value = 1)
sd_spi = SPI(2,
             baudrate = 1000000,
             polarity = 0,
             phase = 0,
             bits = 8,
             firstbit = SPI.MSB,
             sck = Pin(18),
             mosi = Pin(23),
             miso = Pin(19))

# Initialize SD card
vosd = sdcard.SDCard(sd_spi, sd_cs)

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(config.WIFI_SSID, config.WIFI_PWD)

while not wlan.isconnected():
    pass
    #print("Waiting to connect:")
time.sleep(1)

while True:
   
    led_board.value(1)
    time.sleep(1)
    led_board.value(0)
    time.sleep(1)
    print(rtc.PrintTime())