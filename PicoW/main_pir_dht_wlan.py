import machine
from machine import Pin, Timer, ADC, I2C
import network
import utime
import time
import dht

sensor_dht11 = dht.DHT11(machine.Pin(2))
#led_red = Pin(3, Pin.OUT)
led = machine.PWM(machine.Pin(3))
led.freq(1000)
potentiometer = ADC(28)
#buzz    = Pin(5, Pin.OUT)
buzz_pwm = machine.PWM(machine.Pin(5))
sensor_pir = Pin(6, Pin.IN, Pin.PULL_DOWN)
alarm_out  = Pin(7, Pin.OUT)
timer = Timer()
tim_buzzer = Timer()

def buzz_stop(tim_buzzer):
    buzz.value(0)

def pir_handler(pin):
    utime.sleep_ms(100)
    if pin.value():
      print("ALARM! Motion detected!")
      buzz.value(1)
      tim_buzzer.init(mode=Timer.ONE_SHOT, period = 300, callback = buzz_stop)
      alarm_out.toggle()

sensor_pir.irq(trigger=machine.Pin.IRQ_RISING, handler=pir_handler)
sensor_temp = ADC(4)
conversion_factor = 3.3 / (65535)
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect('','')

def blink(timer):
    #led_red.toggle()
    #buzz.toggle()
    led.duty_u16(potentiometer.read_u16())
    buzz_pwm.duty_u16(potentiometer.read_u16())

#buzz.value(0)

timer.init(freq=50, mode=Timer.PERIODIC, callback=blink)

while not wlan.isconnected() and wlan.status() >= 0:
    print("Waiting to connect:")
time.sleep(1)

print(wlan.ifconfig())
# Handle connection error
if wlan.status() != 3:
    raise RuntimeError('network connection failed')
else:
    print('connected')
    status = wlan.ifconfig()
    print( 'ip = ' + status[0] )
    
while True:
    time.sleep(5)
    sensor_dht11.measure()
    dht11_temp = (sensor_dht11.temperature())
    dht11_hum = (sensor_dht11.humidity())
    read_adctemp = sensor_temp.read_u16() * conversion_factor
    adc_temperature = 27 - (read_adctemp - 0.706)/0.001721
    print("Internal tempreature: {}".format(adc_temperature))
    print("Temperature: {}".format(dht11_temp))
    print("Humidity: {}".format(dht11_hum))