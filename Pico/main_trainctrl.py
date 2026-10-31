from machine import Pin, ADC, SPI, UART, Timer
from mcp42010 import MCP42010
import hardware_trainctrl as hw
import pcuart
import time
import asyncio
from asyncio import Event

Led1 = Pin(24, Pin.OUT, value = 0)
Led2 = Pin(25, Pin.OUT, value = 1)


# PC debug UART
pc_uart = UART(0, baudrate = 115200, tx = Pin(0), rx = Pin(1))

pc_rx_data = None   # data received by last IRQ
pc_rx_flag = asyncio.ThreadSafeFlag()  # Flag to indicate new UART RX packet

async def event_PcUartRx():
    global pc_rx_data
    while True:
        await pc_rx_flag.wait()
        print(f"Received {len(pc_rx_data)} bytes from PC: {pc_rx_data}")
        response = pcuart.PcUartProcessInputFrame(pc_rx_data)
        if response:
            pc_uart.write(response)
            print(f"Sent response to PC: {response}")
        pc_rx_flag.clear()

event_100msperiodic = asyncio.Event()
cnt_100msperiodic = 0

async def event_PeriodicTask():
    global cnt_100msperiodic
    while True:
        # Perform periodic tasks here
        await event_100msperiodic.wait()
        cnt_100msperiodic += 1
        hw.adc_values_read()  # Update ADC values
        if cnt_100msperiodic == 10:  # Every 1 second
            cnt_100msperiodic = 0
            Led1.toggle()
            Led2.toggle()
            sensor_temp_value = hw.sensor_temp.read_u16() * hw.conversion_factor
            adc_temperature = 27 - (sensor_temp_value - 0.706)/0.001721
            print("Chip temperature: ", adc_temperature, "°C")
        event_100msperiodic.clear()

# PC UART IRQ handler function
def pc_uart_rx_handler(uart_obj):
    global pc_rx_data, pc_rx_flag
    print("UART IRQ triggered")
    if uart_obj.any():
        pc_rx_data = uart_obj.read()
        pc_rx_flag.set()

def periodic_timer_callback(timer):
    event_100msperiodic.set()
        
timer_led = Timer(mode = Timer.PERIODIC, period = 100, callback = periodic_timer_callback)  # 1000 ms = 1 second

# UART.IRQ_RXIDLE fires once the RX line has been idle after receiving
# data, so the handler runs with the whole message already in the buffer.
pc_uart.irq(handler = pc_uart_rx_handler, trigger=UART.IRQ_RXIDLE )

async def main():
    asyncio.create_task(event_PcUartRx())
    asyncio.create_task(event_PeriodicTask())
    while True:
        await asyncio.sleep(1)

asyncio.run(main())
