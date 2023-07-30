# build-in LED blink in Pi Pico using CircuitPython

import time
import board
import digitalio
import analogio
import usb_midi
import rotaryio
import displayio
import adafruit_ssd1322
import busio

import adafruit_midi
from adafruit_midi.note_on          import NoteOn
from adafruit_midi.note_off         import NoteOff
from adafruit_midi.pitch_bend       import PitchBend
from adafruit_midi.control_change   import ControlChange

class PushButton:
    def __init__(
        self,
        act_value,
        deb_value,
        pin
    ):
        self.active_value = act_value
        self.pin          = pin
        self.counter      = 0
        self.debounce     = deb_value
        self.is_pressed   = False
        self.was_pressed  = False
        
    def update(self, state):
        if self.is_pressed is False:
            if state is self.active_value:
                if self.counter < self.debounce:
                    self.counter += 1
                else:
                    self.is_pressed = True
                    self.counter = 0
            else:
                if self.counter > 0:
                    self.counter -= 1
        else:
            if state is not self.active_value:
                if self.counter < self.debounce:
                    self.counter += 1
                else:
                    self.is_pressed = False
                    self.counter = 0
            else:
                if self.counter > 0:
                    self.counter -= 1
                    
    def checkPushed(self):
        retval = False
        if self.is_pressed is True and self.was_pressed is False:
            self.was_pressed = True
            retval = True
        elif self.is_pressed is False and self.was_pressed is True:
            self.was_pressed = False
        return retval

enc = rotaryio.IncrementalEncoder(board.GP10, board.GP11)
last_position = None

adcPitchBend = analogio.AnalogIn(board.GP28)  
    
led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT # set the direction of the pin

spi = busio.SPI(board.GP18, board.GP19)
tft_cs = board.GP17
tft_dc = board.GP16

display_bus = displayio.FourWire(spi, command=tft_dc, chip_select=tft_cs,
                                     baudrate=1000000)
time.sleep(1)
display = adafruit_ssd1322.SSD1322(display_bus, width=256, height=64, colstart=28)

print("Hello")

midi = adafruit_midi.MIDI(midi_out=usb_midi.ports[1], out_channel=0)

out_pins = [board.GP2, board.GP3, board.GP4, board.GP5, board.GP8]
kd_pins  = [board.GP13, board.GP15, board.GP14]

kd_buttons = []
s_output = []

#KD pins: input pins of key matrix
for pin in kd_pins:
    note_pin = digitalio.DigitalInOut(pin)
    note_pin.direction = digitalio.Direction.INPUT
    kd_buttons.append(note_pin)
    
for pin in out_pins:
    note_pin = digitalio.DigitalInOut(pin)
    note_pin.direction = digitalio.Direction.OUTPUT
    s_output.append(note_pin)
    
#  note states
note0_pressed = False
note1_pressed = False
note2_pressed = False
note3_pressed = False
note4_pressed = False
note5_pressed = False
note6_pressed = False
note7_pressed = False
note8_pressed = False
note9_pressed = False
note10_pressed = False
note11_pressed = False
note12_pressed = False

#  array of note states
note_states_s1 = [note0_pressed, note1_pressed,  note2_pressed]
note_states_s2 = [note3_pressed, note4_pressed,  note5_pressed]
note_states_s3 = [note6_pressed, note7_pressed,  note8_pressed]
note_states_s4 = [note9_pressed, note10_pressed, note11_pressed]

note_states = []
note_states.append(note_states_s1)
note_states.append(note_states_s2)
note_states.append(note_states_s3)
note_states.append(note_states_s4)

note_states_s5 = note12_pressed

#  array of default MIDI notes
midi_notes_s1 = [60, 61, 62]
midi_notes_s2 = [63, 64, 65]
midi_notes_s3 = [66, 67, 68]
midi_notes_s4 = [69, 70, 71]

midi_notes = []
midi_notes.append(midi_notes_s1)
midi_notes.append(midi_notes_s2)
midi_notes.append(midi_notes_s3)
midi_notes.append(midi_notes_s4)

midi_notes_s5 = 72

btn_array_s1 = [PushButton(True, 10, kd_buttons[0]), PushButton(True, 10, kd_buttons[1]), PushButton(True, 10, kd_buttons[2])]
btn_array_s2 = [PushButton(True, 10, kd_buttons[0]), PushButton(True, 10, kd_buttons[1]), PushButton(True, 10, kd_buttons[2])]
btn_array_s3 = [PushButton(True, 10, kd_buttons[0]), PushButton(True, 10, kd_buttons[1]), PushButton(True, 10, kd_buttons[2])]
btn_array_s4 = [PushButton(True, 10, kd_buttons[0]), PushButton(True, 10, kd_buttons[1]), PushButton(True, 10, kd_buttons[2])]
btn_s5 = PushButton(True, 10, kd_buttons[1])

btn_array = []
btn_array.append(btn_array_s1)
btn_array.append(btn_array_s2)
btn_array.append(btn_array_s3)
btn_array.append(btn_array_s4)

pitchCnt = 0
pitchValue = 0
pitchValueLast = 0
last_position = 0

led.value = True

while True:

    #Go through S1 - S4: output of matrix
    for i in range(4):
        s_output[i].value = 1
        
        #check the corresponding button KD0-KD2
        for j in range (3):
            btn_array[i][j].update(kd_buttons[j].value)
            if btn_array[i][j].checkPushed() is True:
                if note_states[i][j] is False:
                    midi.send(NoteOn(midi_notes[i][j], 71))
                    note_states[i][j] = True
                    led.value = True
                    print("Note on:" + str(midi_notes[i][j]))
                else:
                    midi.send(NoteOff(midi_notes[i][j], 71))
                    note_states[i][j] = False
                    led.value = False
                    print("Note off" + str(midi_notes[i][j]))

        s_output[i].value = 0
 
    #WAH button: S5 - KD1
    s_output[4].value = 1
    #  MIDI input
    btn_s5.update(kd_buttons[1].value)
    if btn_s5.checkPushed() is True:
        if note_states_s5 is False:
            midi.send(NoteOn(midi_notes_s5, 71))
            note_states_s5 = True
            led.value = True
            print("Note on:" + str(midi_notes_s5))
        else:
            midi.send(NoteOff(midi_notes_s5, 71))
            note_states_s5 = False
            led.value = False
            print("Note off" + str(midi_notes_s5))
                
    s_output[4].value = 0
    
    pitchCnt += 1
    if pitchCnt == 50:
        pitchValue = adcPitchBend.value
    
        if abs(pitchValue - pitchValueLast) > 400:
            midi.send(PitchBend(pitchValue >> 2))
        
        pitchCnt = 0
    
        pitchValueLast = pitchValue
    
    position = enc.position
    if last_position == None or position != last_position:
        if position > last_position:
            midi.send(ControlChange(11,1))
        else:
            midi.send(ControlChange(11,0x7F))

    last_position = position
    
    time.sleep(0.001)
            
