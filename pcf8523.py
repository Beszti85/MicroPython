from micropython import const

CONTROL_REG_1  = const(0)  # 0x00
CONTROL_REG_2  = const(1)  # 0x01
CONTROL_REG_3  = const(2)  # 0x02
DATETIME_REG   = const(3)  # 0x03-0x09
ALARM_REG_MIN  = const(10) # 0x0A
ALARM_REG_HOUR = const(11) # 0x0B
ALARM_REG_DAY  = const(12) # 0x0C
ALARM_REG_WEEK = const(13) # 0x0D
OFFSET_REG     = const(14) # 0x0E

class PCF8523(object):
    
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
        return f"{self._bcd2dec(buf[6]) + 2000}/{self._bcd2dec(buf[5])}/{self._bcd2dec(buf[4])}\n{self._bcd2dec(buf[2])}:{self._bcd2dec(buf[1])}:{self._bcd2dec(buf[0] & 0x7F)}"

    def DateTime(self, datetime = None):
        """Get or set datetime"""
        if datetime is None:
            buf = [0]*7
            buf = self.i2c.readfrom_mem(self.addr, DATETIME_REG, 7)
            return(buf)
        else:
            self.i2c.writeto_mem(self.addr, DATETIME_REG, datetime)