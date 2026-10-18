import pcdatareadwrite

class PcUartProtIds:
    PCUART_CONNECT    = 0
    PCUART_READ_DATA  = 1
    PCUART_WRITE_DATA = 2
    PCUART_CMD_EXEC   = 3

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
            print("Valid frame received from PC")
            responseFrame.append(0xBE)
            responseFrame.append(0)
            responseFrame.append(0)
            responseFrame.append(0xBE)
            responseFrame.append(payload[4] | 0x80)
            cmdResponse = PcUartProtHandler(payload[4:4+cmdLength])
            #responseFrame.extend(PcUartProtHandler(payload[4:4+cmdLength]))
            responseFrame.extend(cmdResponse)
            responseFrame[1] = len(cmdResponse)
            responseFrame[2] = responseFrame[1]
            responseFrame.append(PcUartCrc8(0, cmdResponse))
            responseFrame.append(0x27)
            print("Response frame to PC: ", responseFrame)
            
    else:
        # invalid frame
        responseFrame.append(0xBE)
        responseFrame.append(0)
        responseFrame.append(0)
        responseFrame.append(0xFF)
        responseFrame.append(0xFF)
        
    return bytearray(responseFrame)
            
def PcUartProtHandler(payload):
    # response buffer
    respBuffer = []
    cmdCode = payload[0]
    # Process the command code
    if cmdCode == PcUartProtIds.PCUART_CONNECT:
        print("PC UART CONNECT command received")
        respBuffer.append(0x12)
    elif cmdCode == PcUartProtIds.PCUART_READ_DATA:
        respBuffer.extend(pcdatareadwrite.PcUartReadDataHandler(payload[1]))
    elif cmdCode == PcUartProtIds.PCUART_WRITE_DATA:
        pcdatareadwrite.PcUartWriteDataHandler(payload[1:])
    elif cmdCode == PcUartProtIds.PCUART_CMD_EXEC:
        pass
        
    return respBuffer
