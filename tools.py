"""
This module shall collect common functions used in hw and sw products
"""
import logging

log = logging.getLogger(__name__)

def log_bytearray(data, level):
    list_data = list(data)
    #log the input based on loglevel
    if level == "ERROR":
        log.error([hex(item) for item in list_data])
    elif level == "INFO":
        log.info([hex(item) for item in list_data])
    elif level == "DEBUG":
        log.debug([hex(item) for item in list_data])
    else:
        print([hex(item for item in list_data)])