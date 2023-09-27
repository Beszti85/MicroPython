# main.py -- put your code here!
import machine
import time
import dht
from machine import Pin
from lcd_pcf8574 import I2cLcd

print("Start")

led_pin = Pin(2, Pin.OUT)
sensor_dht11 = dht.DHT11(Pin(0))
#i2c display
scl_pin = Pin(22)
sda_pin = Pin(21)
pwm_lcd = machine.PWM(Pin(4))

pwm_lcd.freq(1000)
pwm_lcd.duty(512)

i2c_board = machine.I2C(sda = sda_pin, scl = scl_pin, freq = 100000)
lcd_i2c = I2cLcd(i2c_board, 0x20, 4, 20)

lcd_i2c.putstrpos("Hello Python!", 1, 0)

while True:
    led_pin.value(1)
    print("Turning ON the led...")
    time.sleep(1)
    led_pin.value(0)
    print("Turning OFF the led...")
    lcd_i2c.toggle_led_red()
    time.sleep(1)
    sensor_dht11.measure()
    dht11_temp = (sensor_dht11.temperature())
    dht11_hum = (sensor_dht11.humidity())
    print("Temperature: {}".format(dht11_temp))
    print("Humidity: {}".format(dht11_hum))
    int_val  = i2c_board.readfrom(0x20, 1)[0]
    print(int_val)
    print(i2c_board.readfrom(0x20, 1))
    lcd_i2c.toggle_led_yellow()
