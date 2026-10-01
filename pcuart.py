from micropython import const
import pcdatareadwrite

PCUART_CONNECT    = const(0)
PCUART_READ_DATA  = const(1)
PCUART_WRITE_DATA = const(2)
PCUART_CMD_EXEC   = const(3)

def PcUartCrc8(start, payload):
    retval = start & 0xFF
    for index, item in enumerate(payload):
        retval += payload[index]
        retval &= 0xFF
        
    return retval

def PcUartProcessInputFrame(payload):
    # Response array
    responseFrame = []
    # Check frame header
    if ( payload[0] == 0xBE and
         payload[1] == payload[2] and
         payload[3] == payload[0] and
         payload[4 + payload[1] +1] == 0x27 ):
        # save length info:
        cmdLength = payload[1]
        # check CRC
        calcCrcVal = PcUartCrc8(0, payload[4:4+cmdLength])
        if calcCrcVal == payload[4 + cmdLength]:
            responseFrame.append(0xBE)
            responseFrame.append(0)
            responseFrame.append(0)
            responseFrame.append(payload[4] | 0x80)
            responseFrame.append(PcUartProtHandler(payload[4:4+cmdLength]))
            
def PcUartProtHandler(payload):
    # response buffer
    respBuffer = []
    cmdCode = payload[0]
    # Process the command code
    match cmdCode:
        case PCUART_CONNECT:
            respBuffer.append(0x12)
        case PCUART_READ_DATA:
            pcdatareadwrite.PcUartReadDataHandler(payload[1])
        case PCUART_WRITE_DATA:
            pcdatareadwrite.PcUartWriteDataHandler(payload[1:])
        case PCUART_CMD_EXEC:
            pass
        
    return respBuffer
