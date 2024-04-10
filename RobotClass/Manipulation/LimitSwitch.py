#import RPi
#import RPi.GPIO as GPIO
import digitalio

class LimitSwitch():


    #Limit Switch class constructor
    def __init__(self, pin, use_pullup_config=True):
        #Save object data members
        self.pin = digitalio.DigitalInOut(pin)
        self.use_pullup_config = use_pullup_config
        #Set board for the GPIO
        ##############GPIO.setmode(GPIO.BOARD)
        #Set up the specified pin
        ###########GPIO.setup(pin, GPIO.IN)
        self.pin.direction = digitalio.Direction.INPUT





    def is_pressed(self) -> bool:
        '''This function returns True if the LimitSwitch is pressed in and False otherwise.'''

        # if pullup -> pressed is 1; if not pullup -> pressed is 0
        return self.pin.value == self.use_pullup_config
    



    def __del__(self) -> None:
        '''A deconstructor to cleanup the GPIO pins after finished with object instance'''
        #cleanup the pin by making it an input
        self.pin.direction = digitalio.Direction.INPUT