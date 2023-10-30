import machine
import config
from machine import Pin, Timer, ADC, I2C, SPI
import network
import utime
import time
import dht
from ds1307 import DS1307
from lcd_pcf8574 import I2cLcd
from ssd1306 import SSD1306_I2C
import framebuf
from nrf24l01 import NRF24L01
import urequests

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

cs_ledDisp = Pin(22, Pin.OUT, Pin.PULL_UP)

print(i2c_board.scan())

oled = SSD1306_I2C(128, 32, i2c_board)

oled.text("Hello Rasberry", 0, 0)
oled.show()

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

#lcd_i2c = I2cLcd(i2c_board, 0x20)

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
wlan.connect(config.WIFI_SSID, config.WIFI_PWD)

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

# pin definition for the Raspberry Pi Pico:
myPins = {"spi": 0, "miso": 16, "mosi": 19, "sck": 18, "csn": 14, "ce": 15}
# Addresses (little endian)
pipes = (b"\xe1\xf0\xf0\xf0\xf0", b"\xd2\xf0\xf0\xf0\xf0")
print("NRF24L01 transmitter")
csn = Pin(myPins["csn"], mode=Pin.OUT, value=1)
ce = Pin(myPins["ce"], mode=Pin.OUT, value=0)
nrf = NRF24L01(SPI(myPins["spi"]), csn, ce, payload_size=8)
nrf.open_tx_pipe(pipes[0])
nrf.open_rx_pipe(1, pipes[1])
nrf.start_listening()
counter = 0  # Increase the value by 1 with each emission

buf = [0x0F, 0]
    
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
    oled.fill(0)
    oled.text(rtc.PrintTime(), 0, 0)
    oled.text("Temperature: {}".format(dht11_temp), 0, 10)
    oled.text("Humidity: {}".format(dht11_hum), 0, 20)
    oled.show()
    #int_val  = i2c_board.readfrom(0x20, 1)[0]
    #print(int_val)
    #print(i2c_board.readfrom(0x20, 1))
    #lcd_i2c.toggle_led_yellow()
    buf[1] = 0x01
    cs_ledDisp.value(0)
    spi_board.write(bytearray(buf))
    #spi_board.write('\x01')
    cs_ledDisp.value(1)
    
    time.sleep(1)
    buf[1] = 0
    cs_ledDisp.value(0)
    spi_board.write(bytearray(buf))
    #spi_board.write('\x00')
    cs_ledDisp.value(1)
    
    dht_readings = {'field1':dht11_temp, 'field2':dht11_hum} 
    request = urequests.post( 'http://api.thingspeak.com/update?api_key=' + config.WRITE_KEY, json = dht_readings, headers = config.HTTP_HEADERS )  
    request.close() 
    
    time.sleep(4)
