# Read and Write data to/from PC via UART
import sys
if sys.implementation.name == "micropython":
    from main import rtc
else:
    from stubfunctions import rtc

class PcDataReadWriteIds:
    PC_RD_BOARD_ID       = 0
    PC_RD_BME280_PYH_VAL = 1
    PC_RD_ADC_PHY_VALUES = 2
    PC_RD_FLASH_ID       = 3
    PC_RW_RTC_READ_TIME  = 4
    PC_RD_FLASH_READ     = 5

def PcUartReadDataHandler(readId):
    retval = None
    if readId == PcDataReadWriteIds.PC_RD_BOARD_ID:
        retval = bytearray("PicoW_2WD_Car", "utf-8")
        print(" ".join("0x{:02x}".format(b) for b in retval))
    elif readId == PcDataReadWriteIds.PC_RW_RTC_READ_TIME:
        retval = rtc.DateTime()
    elif readId == PcDataReadWriteIds.PC_RD_ADC_PHY_VALUES:
        retval = bytearray([0x00] * 8)
    else:
        # Not used Id: return error code 0xFD
        retval = bytearray([0xFE])
            
    return retval
        
def PcUartWriteDataHandler(payload):
    retval = None
    writeId = payload[0]
    if writeId == PcDataReadWriteIds.PC_RW_RTC_READ_TIME:
        pass
    else:
        # Not used Id: return error code 0xFD
        return bytearray([0xFE])
