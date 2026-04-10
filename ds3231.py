
#REGISTER DEFINITIONS
REG_CONTROL = 0x0E

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
        self.i2c.writeto_mem(self.address, REG_CONTROL, bytearray([reg_value]))