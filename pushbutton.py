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
        
    def update(self, value):
        if self.is_pressed is False:
            if value is self.active_value:
                if self.counter < self.debounce:
                    self.counter += 1
                else:
                    self.is_pressed = True
                    self.counter = 0
            else:
                if self.counter > 0:
                    self.counter -= 1
        else:
            if value is not self.active_value:
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