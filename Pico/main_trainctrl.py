from machine import Pin, ADC, SPI, UART, Timer
from mcp42010 import MCP42010
import time

Led1 = Pin(24, Pin.OUT)
Led2 = Pin(25, Pin.OUT)

# PC debug UART
pc_uart = UART(0, baudrate = 9600, tx = Pin(0), rx = Pin(1))

pc_rx_data = None   # data received by last IRQ
pc_rx_flag = False  # Flag to indicate new UART RX packet

# PC UART IRQ handler function
def pc_uart_rx_handler(uart_obj):
    global pc_rx_data, pc_rx_flag
    if uart_obj.any():
        pc_rx_data = uart_obj.read()
        pc_rx_flag = True

# UART.IRQ_RXIDLE fires once the RX line has been idle after receiving
# data, so the handler runs with the whole message already in the buffer.
pc_uart.irq(handler = pc_uart_rx_handler, trigger=UART.IRQ_RXIDLE )

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
    adc26_value = adc26.read_u16() * conversion_factor
    adc27_value = adc27.read_u16() * conversion_factor
    adc28_value = adc28.read_u16() * conversion_factor
    adc29_value = adc29.read_u16() * conversion_factor
    print("ADC26: ", adc26_value, "V")
    print("ADC27: ", adc27_value, "V")
    print("ADC28: ", adc28_value, "V")
    print("ADC29: ", adc29_value, "V")
    sensor_temp_value = sensor_temp.read_u16() * conversion_factor
    adc_temperature = 27 - (sensor_temp_value - 0.706)/0.001721
    print("Chip temperature: ", adc_temperature, "°C")
    if pc_rx_flag:
        print("Received data from PC: ", pc_rx_data)
        pc_rx_flag = False
    Led1.value(1)
    Led2.value(0)
    time.sleep(1)
    Led1.value(0)
    Led2.value(1)
    time.sleep(1)
