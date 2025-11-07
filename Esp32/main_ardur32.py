# main.py -- put your code here!
import machine
import time
import config
#from pushbutton import PushButton
import network
from machine import Pin, Timer, ADC
import bme280

print("Start")

# on board blue led
led_pin = Pin(2, Pin.OUT)
# rgb led
led_rgbBlue  = Pin(25, Pin.OUT)
led_rgbGreen = Pin(26, Pin.OUT)
led_rgbRed   = Pin(27, Pin.OUT)

led_RGB = [led_rgbBlue, led_rgbGreen, led_rgbRed]

# Timer to refresh button states
button_timer  = Timer(0)
led_rgb_timer = Timer(1)

#i2c bus - BME280 and EEPROM AA24LC256
scl_pin = Pin(22)
sda_pin = Pin(21)

#ADC channels
adc17 = ADC(Pin(35), atten = ADC.ATTN_11DB)
adc16 = ADC(Pin(34), atten = ADC.ATTN_11DB)
adc10 = ADC(Pin(36), atten = ADC.ATTN_11DB)
adc13 = ADC(Pin(39), atten = ADC.ATTN_11DB)

i2c_board = machine.I2C(sda = sda_pin, scl = scl_pin, freq = 200000)
spi_board = machine.SPI(2, 3000000)

time.sleep(2)

print('Scan i2c bus...')
devices = i2c_board.scan()

if len(devices) == 0:
  print("No i2c device !")
else:
  print('i2c devices found:',len(devices))

  for device in devices:  
    print("Decimal address: ",device," | Hexa address: ",hex(device))

#BME280
bme = bme280.BME280(i2c=i2c_board)
time.sleep(5)

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(config.WIFI_SSID, config.WIFI_PWD)

while not wlan.isconnected():
    print("Waiting to connect:")
time.sleep(1)

print("Connected to network!")
print(wlan.ifconfig())

led_rgb_index = 0

def led_rgb_refresh(timer):
    global led_rgb_index
    
    if led_rgb_index is 0:
        led_RGB[0].value(1)
        led_RGB[1].value(0)
        led_RGB[2].value(0)
        print("Led timer: value = ", led_rgb_index)
        led_rgb_index = 1
    elif led_rgb_index is 1:
        led_RGB[0].value(0)
        led_RGB[1].value(1)
        led_RGB[2].value(0)
        print("Led timer: value = ", led_rgb_index)
        led_rgb_index = 2
    elif led_rgb_index is 2:
        led_RGB[0].value(0)
        led_RGB[1].value(0)
        led_RGB[2].value(1)
        print("Led timer: value = ", led_rgb_index)
        led_rgb_index = 0

led_rgb_timer.init(mode = Timer.PERIODIC, period=1000, callback = led_rgb_refresh)

led_index = 0

while True:
       
    led_index = 0    
    led_pin.value(1)

    print("Turning ON the led...")
    time.sleep(1)
    led_pin.value(0)
    print("Turning OFF the led...")
    time.sleep(1)
    print(bme.values)
    #int_val  = i2c_board.readfrom(0x20, 1)[0]
    #print(int_val) 
    #print(i2c_board.readfrom(0x20, 1))
    print('ADC17 value = {}'.format(adc17.read_uv() / 1000000))
    print('ADC16 value = {}'.format(adc16.read_uv() / 1000000))
    print('ADC10 value = {}'.format(adc10.read_uv() / 1000000))
    print('ADC13 value = {}'.format(adc13.read_uv() / 1000000))
    
