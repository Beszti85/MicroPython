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
    
    LCD_START_LINE1     = 0x00  # DDRAM address of first char of line 1
    LCD_START_LINE2     = 0x40  # DDRAM address of first char of line 2
    LCD_START_LINE3     = 0x14  # DDRAM address of first char of line 3
    LCD_START_LINE4     = 0x54  # DDRAM address of first char of line 4

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
        
    def move_to(self, cursor_x, cursor_y):
        # Moves the cursor position to the indicated position. The cursor
        # position is zero based (i.e. cursor_x == 0 indicates first column).
        self.cursor_x = cursor_x
        self.cursor_y = cursor_y
        addr = cursor_x & 0x3f
        if cursor_y & 1:
            addr += 0x40    # Lines 1 & 3 add 0x40
        if cursor_y & 2:    # Lines 2 & 3 add number of columns
            addr += self.num_columns
        self.write_LcdData(self.LCD_DDRAM | addr)

    def putchar(self, char):
        # Writes the indicated character to the LCD at the current cursor
        # position, and advances the cursor by one position.
        if char == '\n':
            if self.implied_newline:
                # self.implied_newline means we advanced due to a wraparound,
                # so if we get a newline right after that we ignore it.
                pass
            else:
                self.cursor_x = self.num_columns
        else:
            self.write_LcdData(ord(char))
            self.cursor_x += 1
        if self.cursor_x >= self.num_columns:
            self.cursor_x = 0
            self.cursor_y += 1
            self.implied_newline = (char != '\n')
        if self.cursor_y >= self.num_lines:
            self.cursor_y = 0
        self.move_to(self.cursor_x, self.cursor_y)

    def putstr(self, string):
        # Write the indicated string to the LCD at the current cursor
        # position and advances the cursor position appropriately.
        for char in string:
            self.putchar(char)
            
    def putstrpos(self, string, line, position):
        # Write the indicated string to the LCD at the current cursor
        # position and advances the cursor position appropriately.
        if line is 1:
            self.write_LcdData(self.LCD_DDRAM + self.LCD_START_LINE1 + position, 0)
        elif line is 2:
            self.write_LcdData(self.LCD_DDRAM + self.LCD_START_LINE2 + position, 0)
        elif line is 3:
            self.write_LcdData(self.LCD_DDRAM + self.LCD_START_LINE3 + position, 0)
        elif line is 4:
            self.write_LcdData(self.LCD_DDRAM + self.LCD_START_LINE4 + position, 0)
        else:
            pass
        
        for char in string:
            self.write_LcdData(ord(char), 1)
        