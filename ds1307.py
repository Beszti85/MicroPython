from micropython import const

DATETIME_REG = const(0) # 0x00-0x06
CHIP_HALT    = const(128)
CONTROL_REG  = const(7) # 0x07
RAM_REG      = const(8) # 0x08-0x3F

class DS1307(object):
    
    def __init__(self, i2c, addr = 0x68):
        self.i2c = i2c
        self.addr = addr
        self.weekday_start = 1 
        self._halt = False
    
    def _dec2bcd(self, value):
        """Convert decimal to binary coded decimal (BCD) format"""
        return (value // 10) << 4 | (value % 10)

    def _bcd2dec(self, value):
        """Convert binary coded decimal (BCD) format to decimal"""
        return ((value >> 4) * 10) + (value & 0x0F)
    
    def PrintTime(self):
        buf = [0]*7
        buf = self.i2c.readfrom_mem(self.addr, DATETIME_REG, 7)
        return f"{self._bcd2dec(buf[6]) + 2000}/{self._bcd2dec(buf[5])}/{self._bcd2dec(buf[4])}\n{self._bcd2dec(buf[2])}:{self._bcd2dec(buf[1])}:{self._bcd2dec(buf[0])}"

    def SquareWave(self, onoff, value):
        reg = 0 | onoff << 4 | value
        self.i2c.writeto_mem(self.addr, CONTROL_REG, bytearray([reg]))
        
    def DateTime(self, datetime = None):
        """Get or set datetime"""
        if datetime is None:
            buf = [0]*7
            buf = self.i2c.readfrom_mem(self.addr, DATETIME_REG, 7)
            return(buf)
        else:
            self.i2c.writeto_mem(self.addr, DATETIME_REG, datetime)
