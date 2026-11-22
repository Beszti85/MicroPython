'''
This module contains the PowerSupply class, which is used to set the output value of a digital potentiometer. The class uses the MCP42010 class to communicate with the potentiometer
The input value is a raw value and shall be converted to the corresponding wiper value of the potentiometer.
This driver shall be independent of the acutal potmeter, shall not use any specific value
Stores the target output voltage and the actual current value, which can be used for monitoring purposes.
'''

class PowerSupply:
    def __init__(self, potmeter, input_range, output_range, max_voltage, minimal_step):
        self.input_max = input_range # in ADC raw value, no voltage codes
        self.output_max = output_range
        self.max_voltage = max_voltage  # in volts
        self.minimal_step = minimal_step  # in raw input value
        self.last_input_value = 0
        self.target_voltage = 0.0
        self.actual_current = 0.0

    def convert_input_to_output(self, input_value):
        # Convert the input value to the corresponding output value
        if input_value < 0 or input_value > self.input_max:
            raise ValueError("Input value out of range")
        output_value = int((input_value / self.input_max) * self.output_max)
        self.target_voltage = (output_value / self.output_max) * self.max_voltage
        if output_value < 0:
            output_value = 0
        elif output_value > self.output_max:
            output_value = self.output_max
        return output_value
    
    def update_output(self, input_value):
        output_value = None
         # Check if the input value has changed by at least the minimal step
        if abs(input_value - self.last_input_value) >= self.minimal_step:
            self.last_input_value = input_value
            # Convert the input value to the corresponding output value and set the potentiometer
            output_value = self.convert_input_to_output(input_value)
        return output_value  # Return the converted output value if new value
    
    