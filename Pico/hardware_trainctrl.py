from machine import Pin, ADC, SPI, UART, Timer
from mcp42010 import MCP42010
from config import DBG_PRINT

#ADC
#adc ports
adc26 = ADC(26)
adc27 = ADC(27)
adc28 = ADC(28)
adc29 = ADC(29)
adc_list = [adc26, adc27, adc28, adc29]
adc_raw_values = [0, 0, 0, 0]
adc_voltages = [0.0, 0.0, 0.0, 0.0]
adc_mv_values = [0, 0, 0, 0]

# Chip temperature sensor
sensor_temp = ADC(4)
conversion_factor = 3.3 / (65535)

#Digit potmeter: MCP42010
CS_MCP42010 = Pin(13, Pin.OUT)
spi1 = SPI(1, baudrate=1000000, polarity=0, phase=0, sck=Pin(10), mosi=Pin(11), miso=Pin(12))
pot_mcp42010 = MCP42010(spi1, CS_MCP42010)

def adc_values_read():
    global adc_raw_values, adc_voltages
    for i in range(4):
        adc_raw_values[i] = adc_list[i].read_u16()
        adc_voltages[i] = adc_raw_values[i] * conversion_factor
        adc_mv_values[i] = int(adc_voltages[i] * 1000)
        if DBG_PRINT:
            print("ADC{}: {} V".format(26+i, adc_voltages[i]))

# ADC reader function
def read_adc_raw_values():
    return adc_raw_values

def read_adc_voltages():
    return adc_voltages

def read_adc_mv_values():
    return adc_mv_values

def read_chip_temperature():
    sensor_temp_value = sensor_temp.read_u16() * conversion_factor
    adc_temperature = 27 - (sensor_temp_value - 0.706)/0.001721
    if DBG_PRINT:
        print("Chip temperature: ", adc_temperature, "°C")
    return adc_temperature

def read_single_adc_value(adc_index):
    if 0 <= adc_index < len(adc_list):
        return adc_list[adc_index].read_u16() * conversion_factor
    else:
        raise ValueError("ADC index out of range. Must be between 0 and 3.")
