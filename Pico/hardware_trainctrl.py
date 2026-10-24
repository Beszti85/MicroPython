from machine import Pin, ADC, SPI, UART, Timer
from mcp42010 import MCP42010

#ADC
#adc ports
adc26 = ADC(26)
adc27 = ADC(27)
adc28 = ADC(28)
adc29 = ADC(29)
adc_list = [adc26, adc27, adc28, adc29]

# Chip temperature sensor
sensor_temp = ADC(4)
conversion_factor = 3.3 / (65535)

#Digit potmeter: MCP42010
CS_MCP42010 = Pin(13, Pin.OUT)
spi1 = SPI(1, baudrate=1000000, polarity=0, phase=0, sck=Pin(10), mosi=Pin(11), miso=Pin(12))
pot_mcp42010 = MCP42010(spi1, CS_MCP42010)

# ADC reader function
def read_adc_values():
    adc_values = []
    for i in range(4):
        adc_value = adc_list[i].read_u16() * conversion_factor
        adc_values.append(adc_value)
    return adc_values

def read_chip_temperature():
    sensor_temp_value = sensor_temp.read_u16() * conversion_factor
    adc_temperature = 27 - (sensor_temp_value - 0.706)/0.001721
    return adc_temperature

def read_single_adc_value(adc_index):
    if 0 <= adc_index < len(adc_list):
        return adc_list[adc_index].read_u16() * conversion_factor
    else:
        raise ValueError("ADC index out of range. Must be between 0 and 3.")