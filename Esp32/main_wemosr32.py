import machine
import config
import network
import ubinascii
from simple import MQTTClient
import time
import bme280

led_board = machine.Pin(2, machine.Pin.OUT)
#i2c bus - BME280 and EEPROM AA24LC256
scl_pin = machine.Pin(22)
sda_pin = machine.Pin(21)
i2c_board = machine.I2C(sda = sda_pin, scl = scl_pin, freq = 200000)
#BME280
bme = bme280.BME280(i2c=i2c_board)

time.sleep(2)

print('Scan i2c bus...')
devices = i2c_board.scan()

if len(devices) == 0:
  print("No i2c device !")
else:
  print('i2c devices found:',len(devices))

client_id = ubinascii.hexlify(machine.unique_id())
client_idhex = machine.unique_id()

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(config.WIFI_SSID, config.WIFI_PWD)

while not wlan.isconnected():
    pass
    #print("Waiting to connect:")
time.sleep(1)

topic_sub = b'notification'
topic_pub = b'domoticz/in'

last_message = 0
message_interval = 5
counter = 0

def sub_cb(topic, msg):
    pass
    #print((topic, msg))
    #if topic == b'notification' and msg == b'received':
        #print('ESP received hello message')

def connect_and_subscribe():
    global client_id, mqtt_server, topic_sub
    client = MQTTClient(client_id, config.mqtt_server, user=config.mqtt_user, password=config.mqtt_pass)
    client.set_callback(sub_cb)
    client.connect()
    client.subscribe(topic_sub)
    #print('Connected to %s MQTT broker, subscribed to %s topic' % (config.mqtt_server, topic_sub))
    return client

def restart_and_reconnect():
    #print('Failed to connect to MQTT broker. Reconnecting...')
    time.sleep(10)
    machine.reset()

try:
    client = connect_and_subscribe()
except OSError as e:
    restart_and_reconnect()

while True:
    #print(bme280_values)
    print(bme.values)
    try:
        client.check_msg()
        if (time.time() - last_message) > message_interval:
            msg = f'{{"command":"udevice", "idx":10, "svalue":"{bme.values_mqtt[0]};{bme.values_mqtt[2]};0;{bme.values_mqtt[1]}"}}'.encode('utf-8')
            client.publish(topic_pub, msg)
            last_message = time.time()
            counter += 1
    except OSError as e:
        restart_and_reconnect()
    
    led_board.value(1)
    time.sleep(1)
    led_board.value(0)
    time.sleep(5)
