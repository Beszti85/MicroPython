import machine, neopixel
import time

color_list = ((255, 255, 255), (255,0,0), (0,255,0), (0,255,255), (255,0,255), (255,255,0), (255, 255, 255))

np1 = neopixel.NeoPixel(machine.Pin(3), 60)
np2 = neopixel.NeoPixel(machine.Pin(15), 60)
np3 = neopixel.NeoPixel(machine.Pin(16), 60)
np4 = neopixel.NeoPixel(machine.Pin(28), 60)
strips = (np1, np2, np3, np4)

def demo(np, colors):
    n = np.n

    # cycle
    for k in range(7):
        for i in range(7 * n):
            for j in range(n):
                np[j] = (0, 0, 0)
            np[i % n] = colors[k]
            np.write()
            time.sleep_ms(25)

    # bounce
    for i in range(4 * n):
        for j in range(n):
            np[j] = (0, 0, 128)
        if (i // n) % 2 == 0:
            np[i % n] = (0, 0, 0)
        else:
            np[n - 1 - (i % n)] = (0, 0, 0)
        np.write()
        time.sleep_ms(60)
        
    # bounce
    for i in range(255):
        for j in range(n):
            np[j] = (i, 0, 128)
        np.write()
        time.sleep_ms(60)

    # fade in/out
    for i in range(0, 4 * 256, 8):
        for j in range(n):
            if (i // 256) % 2 == 0:
                val = i & 0xff
            else:
                val = 255 - (i & 0xff)
            np[j] = (val, 0, 0)
        np.write()

    # clear
    for i in range(n):
        np[i] = (0, 0, 0)
    np.write()
    
def color_wheel(pos):
    #Rainbow colors across 0-255 positions
    if pos < 85:
        retval = (pos * 3, 255 - pos * 3, 0)
    elif pos < 170:
        pos -= 85
        retval = (255 - pos * 3, 0, pos * 3)
    else:
        pos -= 170
        retval = (0, pos * 3, 255 - pos * 3)
        
    return retval

def fade_color(color, fade_value):
    """Fade the color by the given fade value."""
    return tuple(max(0, min(255, int(c * fade_value))) for c in color)

def cycle_colors(wait, num_leds, np):
    #Cycle through the color wheel
    while True:
        for s in range(len(np)+1):
            for i in range(256):
                for j in range(num_leds):
                    np[s][j] = color_wheel((i+j) & 255)
                np[s].write()
                time.sleep_ms(wait)
                    
cycle_colors(20, 60, strips)

 
"""
demo(np1, color_list)
demo(np2, color_list)
demo(np3, color_list)
demo(np4, color_list)"""


