# main.py -- put your code here!
import machine
import time
import config
from pushbutton import PushButton
import network
from machine import Pin, TouchPad, Timer

print("Start")

# on board blue led
led_pin = Pin(2, Pin.OUT)
# rgb led
led_rgbBlue  = Pin(25, Pin.OUT)
led_rgbGreen = Pin(26, Pin.OUT)
led_rgbRed   = Pin(27, Pin.OUT)

led_RGB = [led_rgbBlue, led_rgbGreen, led_rgbRed]

# Pushbuttons
button_gp34 = PushButton(0, 5, Pin(34, Pin.IN))
button_gp39 = PushButton(0, 5, Pin(39, Pin.IN))
# Timer to refresh button states
button_timer = Timer(0)

touch_sens   = TouchPad(Pin(12))
#i2c bus - AHT21+ENS160
scl_pin = Pin(22)
sda_pin = Pin(21)

adc0 = machine.ADC(Pin(36))
adc0.atten(machine.ADC.ATTN_11DB)

i2c_board = machine.I2C(sda = sda_pin, scl = scl_pin, freq = 100000)
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

#AHT21 seems not working on the module
#buf = [0]
#buf = i2c_board.readfrom_mem(0x38, 0, 1)

time.sleep(10)

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(config.WIFI_SSID, config.WIFI_PWD)

while not wlan.isconnected():
    print("Waiting to connect:")
time.sleep(1)

print("Connected to network!")
print(wlan.ifconfig())

def button_refresh(timer):
    button_gp34.update(button_gp34.pin.value())
    button_gp39.update(button_gp39.pin.value())

button_timer.init(mode=Timer.PERIODIC, period=20, callback=button_refresh)

led_index = 0

while True:
    if button_gp34.checkPushed() is True:
        print("GP34 pressed")
    if button_gp39.checkPushed() is True:
        print("GP39 pressed")
        
    led_index = 0    
    led_pin.value(1)
    led_RGB[2].value(0)
    led_RGB[0].value(1)
    print("Turning ON the led...")
    time.sleep(1)
    led_RGB[0].value(0)
    led_RGB[1].value(1)
    led_pin.value(0)
    print("Turning OFF the led...")
    time.sleep(1)
    led_RGB[1].value(0)
    led_RGB[2].value(1)
    time.sleep(1)
    print("Touch value: {}".format(touch_sens.read()))
    #int_val  = i2c_board.readfrom(0x20, 1)[0]
    #print(int_val)
    #print(i2c_board.readfrom(0x20, 1))
    print('ADC0 value = {}'.format(adc0.read()))
