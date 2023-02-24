# main.py -- put your code here!
import machine
import time
import dht

led_pin = machine.Pin(2, machine.Pin.OUT)
sensor_dht11 = dht.DHT11(machine.Pin(0))

while True:
    led_pin.value(1)
    print("Turning ON the led...")
    time.sleep(1)
    led_pin.value(0)
    print("Turning OFF the led...")
    time.sleep(1)
    sensor_dht11.measure()
    dht11_temp = (sensor_dht11.temperature())
    dht11_hum = (sensor_dht11.humidity())
    print("Temperature: {}".format(dht11_temp))
    print("Humidity: {}".format(dht11_hum))