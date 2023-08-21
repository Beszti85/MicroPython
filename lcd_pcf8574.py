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
ENA_MASK  = 0x04
RS_MASK   = 0x08
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
        self.write_datapins(LCD_FUNCTION_RESET)
        self.toggle_enable()
        utime.sleep_ms(6)    # Need to delay at least 4.1 msec
        self.toggle_enable()
        utime.sleep_ms(1)
        self.toggle_enable()
        utime.sleep_ms(1)
        # Put LCD into 4-bit mode
        self.write_datapins(0x20)
        self.toggle_enable()
        utime.sleep_ms(1)
        
    def toggle_led_yellow(self):

        portVal = self.i2c.readfrom(self.i2c_addr, 1)[0]
        if portVal & LED_YELLOW_MASK :
            portVal &= ~LED_YELLOW_MASK
        else:
            portVal |= LED_YELLOW_MASK

        self.i2c.writeto(self.i2c_addr, bytes([portVal]))
    
    def toggle_led_red(self):

        portVal = self.i2c.readfrom(self.i2c_addr, 1)[0]
        if portVal & LED_RED_MASK :
            portVal &= ~LED_RED_MASK
        else:
            portVal |= LED_RED_MASK

        self.i2c.writeto(self.i2c_addr, bytes([portVal]))

    def write_datapins(self, data):
        portVal = self.i2c.readfrom(self.i2c_addr, 1)[0]
        data &= DATA_MASK
        self.i2c.writeto(self.i2c_addr, bytes([portVal]))
        
    def toggle_enable(self):
        portVal = self.i2c.readfrom(self.i2c_addr, 1)[0]
        portVal |= ENA_MASK
        self.i2c.writeto(self.i2c_addr, bytes([portVal]))
        portVal &= ~ENA_MASK
        self.i2c.writeto(self.i2c_addr, bytes([portVal]))

    # Set RS pin value
    def write_rspin(self, data):
        portVal = self.i2c.readfrom(self.i2c_addr, 1)[0]
        if data is 1:
            portVal |= RS_MASK
        else:
            portVal &= ~RS_MASK
        self.i2c.writeto(self.i2c_addr, bytes([portVal]))

    # Converts LCD char data to 4bit data
    def write_LcdData(self, data, rs):
        tempData = 0
        if rs is 1:
            write_rspin(1)
        else:
            write_rspin(0)
        #Set data port value: High nibble   
        if data & 0x80:
            tempData |= (1 << PIN_D7)
        if data & 0x40:
            tempData |= (1 << PIN_D6)
        if data & 0x20:
            tempData |= (1 << PIN_D5)
        if data & 0x10:
            tempData |= (1 << PIN_D4)
        #Write data pins
        self.write_datapins(tempData)
        self.toggle_enable()
        #Set data port value: High nibble   
        if data & 0x08:
            tempData |= (1 << PIN_D7)
        if data & 0x04:
            tempData |= (1 << PIN_D6)
        if data & 0x02:
            tempData |= (1 << PIN_D5)
        if data & 0x01:
            tempData |= (1 << PIN_D4)
        #Write data pins
        self.write_datapins(tempData)
        self.toggle_enable()
        
        utime.sleep_ms(1)

