

class Signal:
    def __init__(
        self,
        value_type,
        value_init,
        value_min,
        value_max,
        value_err,
        debounce_min,
        debounce_max,
    ):
        self.initValue = value_init
        self.minValue  = value_min
        self.maxValue  = value_max
        self.errValue  = value_err
        self.actValue  = value_init
        self.actValue  = 0
        self.debounce_min = debounce_min
        self.debounce_max = debounce_max
        