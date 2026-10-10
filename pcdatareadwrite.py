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
    match readId:
        case PcDataReadWriteIds.PC_RD_BOARD_ID:
            retval = bytearray("PicoW_2WD_Car", "utf-8")
            print(" ".join(f"0x{b:02x}" for b in retval))
        case PcDataReadWriteIds.PC_RW_RTC_READ_TIME:
            retval = rtc.DateTime()
        case PcDataReadWriteIds.PC_RD_ADC_PHY_VALUES:
            retval = bytearray([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        case _:
            # Not used Id: return error code 0xFD
            retval = bytearray([0xFE])
            
    return retval
        
def PcUartWriteDataHandler(payload):
    retval = None
    writeId = payload[0]
    match writeId:
        case PcDataReadWriteIds.PC_RW_RTC_READ_TIME:
            pass
        case _:
            # Not used Id: return error code 0xFD
            retval = bytearray([0xFE])