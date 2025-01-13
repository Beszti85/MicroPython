import machine
from time import sleep

led_board = machine.Pin(2, machine.Pin.OUT)

while True:
    led_board.value(1)
    sleep(1)
    led_board.value(0)
    sleep(1)