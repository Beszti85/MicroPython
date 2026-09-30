from micropython import const

PC_RD_BOARD_ID       = const(0)
PC_RD_BME280_PYH_VAL = const(1)
PC_RD_ADC_PHY_VALUES = const(2)
PC_RD_FLASH_ID       = const(3)
PC_RW_RTC_READ_TIME  = const(4)
PC_RD_FLASH_READ     = const(5)

def PcUartReadDataHandler(readId):
    retval = None
    match readId:
        case PC_RD_BOARD_ID:
            retval = "PicoW_2WD_Car"
        case PC_RW_RTC_READ_TIME:
            retval = rtc.DateTime()
            
    return retval
        
def PcUartWriteDataHandler(payload):
    retval = None
    writeId = payload[0]
    match writeId:
        case PC_RW_RTC_READ_TIME:
            pass