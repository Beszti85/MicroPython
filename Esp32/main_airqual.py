# main.py -- put your code here!
import machine
import time
import config
from pushbutton import PushButton
import network
from machine import Pin, TouchPad, Timer
import aht
from ssd1306 import SSD1306_SPI
import framebuf
import ubinascii
from simple import MQTTClient

print("Start")

client_id = ubinascii.hexlify(machine.unique_id())

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
button_timer  = Timer(0)
led_rgb_timer = Timer(1)

touch_sens   = TouchPad(Pin(12))
#i2c bus - AHT21+ENS160
scl_pin = Pin(22)
sda_pin = Pin(21)

# Photoresistor on ADC0
adc0 = machine.ADC(Pin(36))
adc0.atten(machine.ADC.ATTN_11DB)
# Input voltage on ADC5
adc5 = machine.ADC(Pin(33))
adc5.atten(machine.ADC.ATTN_11DB)

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

time.sleep(5)

#Setup AHT21
sensor_aht21 = aht.AHT2x(i2c_board, crc=False)

#display pins
cs = Pin(16)
rst = Pin(0)
dc = Pin(4)

#display init
oled = SSD1306_SPI(128, 64, spi_board, dc, rst, cs)

oled.text("Hello Rasberry", 0, 0)
oled.show()

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

button_timer.init(mode = Timer.PERIODIC, period=20, callback = button_refresh)

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

topic_sub = b'notification'
topic_pub = b'hello'

last_message = 0
message_interval = 5
counter = 0

def sub_cb(topic, msg):
  print((topic, msg))
  if topic == b'notification' and msg == b'received':
    print('ESP received hello message')

def connect_and_subscribe():
  global client_id, mqtt_server, topic_sub
  client = MQTTClient(client_id, config.mqtt_server, user=config.mqtt_user, password=config.mqtt_pass)
  client.set_callback(sub_cb)
  client.connect()
  client.subscribe(topic_sub)
  print('Connected to %s MQTT broker, subscribed to %s topic' % (config.mqtt_server, topic_sub))
  return client

def restart_and_reconnect():
  print('Failed to connect to MQTT broker. Reconnecting...')
  time.sleep(10)
  machine.reset()

try:
  client = connect_and_subscribe()
except OSError as e:
  restart_and_reconnect()

led_rgb_timer.init(mode = Timer.PERIODIC, period=1000, callback = led_rgb_refresh)

led_index = 0

while True:
    if button_gp34.checkPushed() is True:
        print("GP34 pressed")
        #stop the RGB led timer
        led_rgb_timer.deinit()
        led_RGB[0].value(0)
        led_RGB[1].value(0)
        led_RGB[2].value(0)
    if button_gp39.checkPushed() is True:
        print("GP39 pressed")
        #start the timer if not running
        led_rgb_timer.init(mode = Timer.PERIODIC, period=1000, callback = led_rgb_refresh)
        
    led_index = 0    
    led_pin.value(1)

    if sensor_aht21.is_ready:
        print("Humidity: {:.2f}".format(sensor_aht21.humidity))
        print("Temperature: {:.2f}".format(sensor_aht21.temperature))
        oled.fill(0)
        oled.text("Temperature: {}".format(sensor_aht21.temperature), 0, 10)
        oled.text("Humidity: {}".format(sensor_aht21.humidity), 0, 20)
        oled.show()

    print("Turning ON the led...")
    time.sleep(1)
    led_pin.value(0)
    print("Turning OFF the led...")
    time.sleep(1)
    print("Touch value: {}".format(touch_sens.read()))
    #int_val  = i2c_board.readfrom(0x20, 1)[0]
    #print(int_val)
    #print(i2c_board.readfrom(0x20, 1))
    print('ADC0 value = {}'.format(adc0.read()))
    print('ADC5 value = {}'.format(adc5.read()))
    
    try:
        client.check_msg()
        if (time.time() - last_message) > message_interval:
            msg = b'Hello #%d' % counter
            client.publish(topic_pub, msg)
            last_message = time.time()
            counter += 1
    except OSError as e:
        restart_and_reconnect()
