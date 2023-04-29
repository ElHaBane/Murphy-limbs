#Libraries
import time
import Constants.variables as var
from gpio_initialization import *

class Limb:
    # sets up the pins when the limb's object is constructed
    '''pins_and_direction represents the pairing of the GPIO pin # and the pin's mode/direction
    ex.*pins_and_directions = [(3, GPIO.OUT), (7, GPIO.IN), (3, GPIO.OUT)]'''
    def __init__(self, *pins_and_directions) -> None:
        self.pins, self.directions = zip(*pins_and_directions)

        for pair in pins_and_directions:
            GPIO.setup(pair[0], pair[1])
    
    # Cleans up the pins when the limb's object is deconstructed
    def __del__(self) -> None:
        for p in self.pins: 
            GPIO.cleanup(p)


class Motors(Limb):
    def __init__(self) -> None:
        super().__init__(
            (var.R_ForwardSignal, GPIO.OUT), 
            (var.R_ReverseSignal, GPIO.OUT), 
            (var.L_ForwardSignal, GPIO.OUT), 
            (var.L_ReverseSignal, GPIO.OUT)
            )

    def stop_motors(self):
        for p in self.pins: GPIO.output(p, GPIO.LOW)

    def drive_forward(self):
        self.stop_motors()
        GPIO.output(var.R_ForwardSignal, GPIO.HIGH)
        GPIO.output(var.L_ForwardSignal, GPIO.HIGH)

    def drive_backwards(self):
        self.stop_motors()
        GPIO.output(var.R_ReverseSignal, GPIO.HIGH)
        GPIO.output(var.L_ReverseSignal, GPIO.HIGH)

    def drive_left(self):
        self.stop_motors()
        GPIO.output(var.L_ForwardSignal, GPIO.HIGH)

    def drive_right(self):
        self.stop_motors()
        GPIO.output(var.R_ForwardSignal, GPIO.HIGH)


class LEDs(Limb):
    def __init__(self) -> None:
        super().__init__(
            (var.L_Window_LED, GPIO.OUT), 
            (var.R_Eye_LED, GPIO.OUT), 
            (var.R_Window_LED, GPIO.OUT), 
            (var.L_Eye_LED, GPIO.OUT)
            )

    def all_high(self):
        for p in self.pins: GPIO.output(p, GPIO.HIGH)

    def all_low(self):
        for p in self.pins: GPIO.output(p, GPIO.LOW)

    # turns on the headlights
    def window_high(self):
        self.all_low()
        GPIO.output(var.L_Window_LED)
        GPIO.output(var.R_Window_LED)
    
    # turns on the backlights
    def eye_high(self):
        self.all_low()
        GPIO.output(var.L_Eye_LED)
        GPIO.output(var.R_Eye_LED)


# Defines the functionality of the Ultra Sonic Sensors
class USSensors(Limb):
    def __init__(self) -> None:
        super().__init__(
            (var.B_US_Echo, GPIO.IN), 
            (var.B_US_Trig, GPIO.OUT), 
            (var.F_US_Echo, GPIO.IN), 
            (var.F_US_Trig, GPIO.OUT), 
            (var.L_US_Echo, GPIO.IN), 
            (var.L_US_Trig, GPIO.OUT), 
            (var.R_US_Echo, GPIO.IN), 
            (var.R_US_Trig, GPIO.OUT)
            )

    #This function is supposed to convert time into distance so that we can get accurate measurements.
    # TODO: integrate distance function into class
    def distance(trig_pin, echo_pin, measure='cm'):
        try:
            GPIO.setup(trig_pin, GPIO.OUT)
            GPIO.setup(echo_pin, GPIO.IN)
            GPIO.output(trig_pin, False)
            while GPIO.input(var.F_US_Echo) == 0:
                nosig = time.time()
            while GPIO.input(var.F_US_Echo) == 1:
                sig = time.time()
            time_lap = sig - nosig
            if measure == 'cm':
                distance = time_lap / 0.000058
            elif measure == 'in':
                distance = time_lap / 0.000148
            else:
                print('improper choice of measurement: in or cm')
                distance = None
                GPIO.cleanup()
                return distance
        except:
            distance = 100
            GPIO.cleanup()
            return distance
        

# TODO: Develope class
class HuskyLens(Limb):
    pass