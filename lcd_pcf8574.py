import utime
from machine import I2C

PIN_D7 = 7
PIN_D6 = 6
PIN_D5 = 5
PIN_D4 = 4
PIN_E  = 2
PIN_RS = 3
PIN_LED_YELLOW = 1
PIN_LED_RED    = 0
DATA_MASK = 0xF0
LED_YELLOW_MASK = 0x02
LED_RED_MASK    = 0x01
LCD_FUNCTION_RESET = 0x30

I2C_ADDR = 0x20

class I2cLcd(object):
    
    def __init__(self, i2c, i2c_addr):
        self.i2c = i2c
        self.i2c_addr = i2c_addr
        self.i2c.writeto(self.i2c_addr, bytes([0]))
        utime.sleep_ms(50)   # Allow LCD time to powerup
        # Send reset 3 times
        self.hal_write_datapins(LCD_FUNCTION_RESET)
        utime.sleep_ms(6)    # Need to delay at least 4.1 msec
        # Put LCD into 4-bit mode
        self.hal_write_datapins(0x20)
        utime.sleep_ms(1)
        
    def toggle_led_yellow(self):

        portVal = self.i2c.readfrom(self.i2c_addr, 1)[0]
        if portVal & LED_YELLOW_MASK :
            portVal &= ~LED_YELLOW_MASK
        else:
            portVal |= LED_YELLOW_MASK

        self.i2c.writeto(self.i2c_addr, bytes([portVal]))

    def hal_write_datapins(self, data):
        portVal = self.i2c.readfrom(self.i2c_addr, 1)[0]
        data &= DATA_MASK
        print(portVal)
        self.i2c.writeto(self.i2c_addr, bytes([portVal]))

