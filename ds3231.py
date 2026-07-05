
#REGISTER DEFINITIONS
DATETIME_REG = const(0) # 0x00-0x06
CHIP_HALT    = const(128)
CONTROL_REG  = const(14) # 0x0E
RAM_REG      = const(8) # 0x08-0x3F

#SQUAREWAVE OUTPUT SETTINGS
SQW_1kHZ    = 0
SQW_1024HZ  = 1
SQW_4096HZ  = 2
SQW_8192HZ  = 3
SQW_DISABLE = 4

class DS3231(object):
    
    def __init__(self, i2c, address = 0x68):
        self.i2c = i2c
        self.address = address
        
    def SquarWaveOutput(self, value):
        #enable the output
        if value < SQW_DISABLE:
            reg_value = 0 | value << 3
        #disable the output
        else:
            reg_value = 0x04
        #write the value
        self.i2c.writeto_mem(self.address, CONTROL_REG, bytearray([reg_value]))
        
    def _dec2bcd(self, value):
        """Convert decimal to binary coded decimal (BCD) format"""
        return (value // 10) << 4 | (value % 10)

    def _bcd2dec(self, value):
        """Convert binary coded decimal (BCD) format to decimal"""
        return ((value >> 4) * 10) + (value & 0x0F)
    
    def PrintTime(self):
        buf = [0]*7
        buf = self.i2c.readfrom_mem(self.address, DATETIME_REG, 7)
        return f"{self._bcd2dec(buf[6]) + 2000}/{self._bcd2dec(buf[5])}/{self._bcd2dec(buf[4])}\n{self._bcd2dec(buf[2])}:{self._bcd2dec(buf[1])}:{self._bcd2dec(buf[0])}"

    def SquareWave(self, onoff, value):
        reg = 0 | onoff << 4 | value
        self.i2c.writeto_mem(self.address, CONTROL_REG, bytearray([reg]))
        
    def DateTime(self, datetime = None):
        """Get or set datetime"""
        if datetime is None:
            buf = [0]*7
            buf = self.i2c.readfrom_mem(self.address, DATETIME_REG, 7)
            return(buf)
        else:
            self.i2c.writeto_mem(self.address, DATETIME_REG, datetime)
