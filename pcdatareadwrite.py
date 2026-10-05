# Read and Write data to/from PC via UART

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
            retval = "PicoW_2WD_Car"
        case PcDataReadWriteIds.PC_RW_RTC_READ_TIME:
            retval = rtc.DateTime()
            
    return retval
        
def PcUartWriteDataHandler(payload):
    retval = None
    writeId = payload[0]
    match writeId:
        case PcDataReadWriteIds.PC_RW_RTC_READ_TIME:
            pass