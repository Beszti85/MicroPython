from machine import Pin, ADC, SPI, UART, Timer
from mcp42010 import MCP42010
import time

Led1 = Pin(24, Pin.OUT)
Led2 = Pin(25, Pin.OUT)

#ADC
#adc ports
adc26 = ADC(26)
adc27 = ADC(27)
adc28 = ADC(28)
adc29 = ADC(29)
# Chip temperature sensor
sensor_temp = ADC(4)
conversion_factor = 3.3 / (65535)

#Digit potmeter: MCP42010
CS_MCP42010 = Pin(13, Pin.OUT)
spi1 = SPI(1, baudrate=1000000, polarity=0, phase=0, sck=Pin(10), mosi=Pin(11), miso=Pin(12))
pot_mcp42010 = MCP42010(spi1, CS_MCP42010)

while(1):
    Led1.value(1)
    Led2.value(0)
    time.sleep(1)
    Led1.value(0)
    Led2.value(1)
    time.sleep(1)
