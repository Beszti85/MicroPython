from machine import Pin, ADC, SPI, UART, Timer
from mcp42010 import MCP42010
import hardware_trainctrl as hw
import pcuart
import time

Led1 = Pin(24, Pin.OUT)
Led2 = Pin(25, Pin.OUT)

# PC debug UART
pc_uart = UART(0, baudrate = 115200, tx = Pin(0), rx = Pin(1))

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

while(1):
    hw.adc_values_read()  # Update ADC values
    sensor_temp_value = hw.sensor_temp.read_u16() * hw.conversion_factor
    adc_temperature = 27 - (sensor_temp_value - 0.706)/0.001721
    #print("Chip temperature: ", adc_temperature, "°C")
    if pc_rx_flag:
        print(f"Received {len(pc_rx_data)} bytes from PC: {pc_rx_data}")
        response = pcuart.PcUartProcessInputFrame(pc_rx_data)
        if response:
            pc_uart.write(response)
            print(f"Sent response to PC: {response}")
        pc_rx_flag = False
    Led1.value(1)
    Led2.value(0)
    time.sleep(1)
    Led1.value(0)
    Led2.value(1)
    time.sleep(1)
