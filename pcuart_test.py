import pcuart
import logging
from datetime import datetime
import tools
import os

START_BYTE = 0xBE
STOP_BYTE  = 0x27

def PcUartSendReadRequest(id):
    request = [1, id]
    pc_payload = CreatePcPayload(request)
    return pcuart.PcUartProcessInputFrame(pc_payload)

def PcUartSendCmdRequest(id):
    request = [3, id]
    pc_payload = CreatePcPayload(request)
    return pcuart.PcUartProcessInputFrame(pc_payload)

def CreatePcPayload(request):
    payload_length = len(request)
    retval = bytearray(6 + payload_length)
    # first byte: start byte - 0xBE
    retval[0] = START_BYTE
    # second and third byte: payload length
    retval[1] = payload_length
    retval[2] = payload_length
    # fourth byte: same as start byte
    retval[3] = START_BYTE
    # copy payload
    retval[4:4+payload_length] = request.copy()
    # penultimate byte: CRC
    retval[4 + payload_length] = pcuart.PcUartCrc8(0, retval[4:4+payload_length])
    # last byte: stop byte
    retval[5 + payload_length] = STOP_BYTE
    #log the payload
    tools.log_bytearray(retval, "DEBUG")
    return retval

def main():
    #create Logs folder if it does not exist
    if not os.path.exists("Logs"):
        os.makedirs("Logs")
    
    log = logging.getLogger(__name__)
    log_filename = datetime.now().strftime("Logs/logfile_%Y%m%d_%H%M%S.log")

    logging.basicConfig(filename=log_filename, level=logging.DEBUG, format = '%(asctime)s - %(levelname)s - %(message)s')
    
    # default payload
    tools.log_bytearray(PcUartSendReadRequest(0), "DEBUG")
    tools.log_bytearray(PcUartSendReadRequest(4), "DEBUG")
    tools.log_bytearray(PcUartSendReadRequest(2), "DEBUG")

if __name__ == "__main__":
    main()