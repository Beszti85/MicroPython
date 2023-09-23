import utime
#from lcd_pcf8574 import I2cLcd

class LcdChar:

    # HD44780 LCD controller command set
    LCD_CLR             = 0x01  # DB0: clear display
    LCD_HOME            = 0x02  # DB1: return to home position

    LCD_ENTRY_MODE      = 0x04  # DB2: set entry mode
    LCD_ENTRY_INC       = 0x02  # DB1: increment
    LCD_ENTRY_SHIFT     = 0x01  # DB0: shift

    LCD_ON_CTRL         = 0x08  # DB3: turn lcd/cursor on
    LCD_ON_DISPLAY      = 0x04  # DB2: turn display on
    LCD_ON_CURSOR       = 0x02  # DB1: turn cursor on
    LCD_ON_BLINK        = 0x01  # DB0: blinking cursor

    LCD_MOVE            = 0x10  # DB4: move cursor/display
    LCD_MOVE_DISP       = 0x08  # DB3: move display (0-> move cursor)
    LCD_MOVE_RIGHT      = 0x04  # DB2: move right (0-> left)

    LCD_FUNCTION        = 0x20  # DB5: function set
    LCD_FUNCTION_8BIT   = 0x10  # DB4: set 8BIT mode (0->4BIT mode)
    LCD_FUNCTION_2LINES = 0x08  # DB3: two lines (0->one line)
    LCD_FUNCTION_10DOTS = 0x04  # DB2: 5x10 font (0->5x7 font)
    LCD_FUNCTION_RESET  = 0x30  # See "Initializing by Instruction" section

    LCD_CGRAM           = 0x40  # DB6: set CG RAM address
    LCD_DDRAM           = 0x80  # DB7: set DD RAM address

    LCD_RS_CMD          = 0
    LCD_RS_DATA         = 1

    LCD_RW_WRITE        = 0
    LCD_RW_READ         = 1

    #constructor
    def __init__(self, num_lines, num_columns):
        self.num_lines = num_lines
        self.num_columns = num_columns
        # limit the maximum of lines to 4
        if self.num_lines > 4:
            self.num_lines = 4
        # limit the maximum of columns to 40
        if self.num_columns > 40:
            self.num_columns = 40
        self.cursor_x = 0
        self.cursor_y = 0
        # Set default mode
        self.write_LcdData(self.LCD_FUNCTION | self.LCD_FUNCTION_2LINES, 0)
        utime.sleep_ms(5)
        self.write_LcdData(self.LCD_ON_CTRL, 0)
        self.clear()
        self.write_LcdData(0xF, 0)
        
    def clear(self):
        # Clears the LCD display and moves the cursor to the top left corner
        self.write_LcdData(self.LCD_CLR, 0)
        #self.write_LcdData(self.LCD_HOME, 0)
        self.cursor_x = 0
        self.cursor_y = 0
        
        
        