# Demo circuitpython project on MakerPico board
import time
import board
from digitalio import DigitalInOut
import busio as io
import adafruit_ssd1306
import adafruit_dht
import analogio
import sdcardio
import storage
from ds1307_cp import DS1307

#sd = sdcardio.SDCard(board.SPI(), board.SD_CS)
#vfs = storage.VfsFat(sd)
#storage.mount(vfs, '/sd')
#19os.listdir('/sd')

# Initialize OLED
# I2C: SCL = GP1, SDA = GP0
# OLED: 128x64 pixels
i2c = io.I2C(board.GP7, board.GP6)
oled = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c)

# Initialize DHT11 sensor
dht_pin = board.GP1
dht_device = adafruit_dht.DHT11(dht_pin)

# Initialize the light sensor
photocell = analogio.AnalogIn(board.A1)

# Initialize DS1307 RTC
rtc_clk = DS1307(i2c)
#rtc_clk.PrintTime()

# Program loop
while True:
    # Clear all pixels
    oled.fill(0)

    try:
        # Read temperature and humidity
        temperature = dht_device.temperature
        humidity = dht_device.humidity
        print(photocell.value)
        #temperature = 25
        #humidity = 10

        # Display temperature and humidity
        oled.text('Temp: {:.1f} C'.format(temperature), 0, 0, 1)
        oled.text('Humidity: {:.1f} %'.format(humidity), 0, 24, 1)

        # Show on OLED
        oled.show()
    except RuntimeError as e:
        # Error occurred, display error message
        oled.text('Error:', 0, 0, 1)
        oled.text(str(e), 0, 12, 1)
        oled.show()

    # Delay before the next reading
    time.sleep(2)


