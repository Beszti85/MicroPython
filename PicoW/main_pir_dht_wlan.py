import machine
from machine import Pin, Timer, ADC, I2C, SPI
import network
import utime
import time
import dht
from ds1307 import DS1307

sensor_dht11 = dht.DHT11(machine.Pin(2))
#led_red = Pin(3, Pin.OUT)
led = machine.PWM(machine.Pin(3))
led.freq(1000)
potentiometer = ADC(28)
#buzz      = Pin(5, Pin.OUT)
buzz_pwm   = machine.PWM(machine.Pin(5))
sensor_pir = Pin(6, Pin.IN, Pin.PULL_DOWN)
alarm_out  = Pin(7, Pin.OUT)
lcd_pwm    = machine.PWM(machine.Pin(8))
lcd_pwm.freq(1000)
lcd_pwm.duty_u16(50000)
i2c_board  = I2C(0, scl = Pin(21), sda = Pin(20), freq = 100000)
spi_board  = SPI(0, sck = Pin(18), mosi = Pin(19), miso = Pin(16), baudrate = 1000000)

#result = i2c_board.scan()
#print(result)
#result = i2c_board.readfrom_mem(0x68, 0x00, 8)
#print(result)
datetime_now = [0x00, 0x26, 0x18, 0x04, 0x18, 0x05, 0x23]
print(datetime_now)

rtc = DS1307(i2c_board)
#rtc.DateTime(bytearray(datetime_now))
rtcTime = rtc.DateTime()
print([hex(x) for x in rtcTime])

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
    pwm_duty = potentiometer.read_u16()
    led.duty_u16(pwm_duty)
    buzz_pwm.duty_u16(pwm_duty)
    lcd_pwm.duty_u16(pwm_duty)

#buzz.value(0)

timer.init(freq=50, mode=Timer.PERIODIC, callback=blink)

while not wlan.isconnected() and wlan.status() >= 0:
    print("Waiting to connect:")
time.sleep(1)

print(wlan.ifconfig())
# Handle connection error
if wlan.status() != 3:
    print('network connection failed')
else:
    print('connected')
    status = wlan.ifconfig()
    print( 'ip = ' + status[0] )
    
while True:
    sensor_dht11.measure()
    dht11_temp = (sensor_dht11.temperature())
    dht11_hum = (sensor_dht11.humidity())
    read_adctemp = sensor_temp.read_u16() * conversion_factor
    adc_temperature = 27 - (read_adctemp - 0.706)/0.001721
    print([hex(x) for x in rtc.DateTime()])
    print(rtc.PrintTime())
    print("Internal tempreature: {}".format(adc_temperature))
    print("Temperature: {}".format(dht11_temp))
    print("Humidity: {}".format(dht11_hum))
    int_val  = i2c_board.readfrom(0x20, 1)[0]
    print(int_val)
    print(i2c_board.readfrom(0x20, 1))
    #lcd_i2c.toggle_led_yellow()
    
    time.sleep(5)
