#
# MCP42xxx family of digital potentiometers
#

class MCP42010:
    def __init__(self, spi, cs):
        self.spi = spi
        self.cs = cs
        self.cs.value(1)  # Set CS high to deselect the device
        self.cmd_data = bytearray(2)  # Command and data bytes

    def set_wiper(self, wiper, pot, nop = False):
        # send the wiper value to the given Pot or both Pots
        if wiper not in [0, 255]:
            raise ValueError("Wiper must be 0 or 255")
        # set command byte based on the selected potmeter
        if pot == 0:           #write to pot 0
            command = 0x11
        elif pot == 1:         #write to pot 1
            command = 0x12
        elif pot == 2:         #write to both pots
            command = 0x13
        else:
            raise ValueError("Pot must be 0 or 2")

        self.cmd_data[0] = command
        self.cmd_data[1] = wiper
        self.spi_write()
        
    def spi_write(self):
        self.cs.value(0)  # Select the device
        self.spi.write(self.cmd_data)
        self.cs.value(1)  # Deselect the device