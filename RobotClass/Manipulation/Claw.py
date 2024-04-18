import time
import numpy as np
from Manipulation.LimitSwitch import LimitSwitch
from Manipulation.Servo import Servo

class Claw():


    #Claw class constructor
    def __init__(self, servo_pin, right_limit_switch_pin, left_limit_switch_pin):
        #Initialize the object data members
        self.percent_open = None
        self.open_bound_deg = -85
        self.close_bound_deg = 5
        #Instantiate the composed objects
        self.right_limit_switch = LimitSwitch(pin=right_limit_switch_pin, use_pullup_config=True)
        self.left_limit_switch = LimitSwitch(pin=left_limit_switch_pin, use_pullup_config=True)
        self.servo = Servo(pin=servo_pin, reset_servo_position=False)
        self.set_percent_open(50)




    def set_percent_open(self, pct_open: float) -> None:
        '''Sets the percent open of the claw on the Cuttlebot.
        
        This function calculates the angle needed to move the claw's servo to achieve
        the given percentage of openness. It then moves the servo accordingly and 
        updates the the Claw's percent_open attribute. The argument pct_open should be
        passed as a float between 0 and 100.'''

        #Check to avoid recomputation
        if pct_open == self.percent_open:
            return
        #Boundary Check
        if pct_open < 0:
            pct_open = 0
        elif pct_open > 100:
            pct_open = 100
        #Compute the angle to move the claw's servo
        angle = self.close_bound_deg + (self.open_bound_deg - self.close_bound_deg)*(pct_open/100.0)
        self.servo.set_angle_deg(angle)
        
        if(self.percent_open == None):
            delay = 2 #max delay possible (moving from 0% <-> 100%)
        else:
            delta_pct_open = pct_open - self.percent_open
            delay = np.abs(delta_pct_open)/50.0 #about 50%/sec movement speed
            delay = min(delay*1.5, 2.0)

        #print(angle, delay)
        time.sleep(delay)
        self.percent_open = pct_open
        #self.servo.stop_signal()



    def get_percent_open(self) -> float:
        '''This function returns the current percentage that the claw is open.'''
        return self.percent_open
    



    def is_object_captured(self) -> bool:
        '''This function looks at the states of the right and left limit switches and
        returns True if both are pressed and False otherwise.'''
        return self.right_limit_switch.is_pressed() or self.left_limit_switch.is_pressed()
    



    def is_object_fully_released(self) -> bool:
        '''This function looks at the states of the right and left limit switches and
        returns True if neither are pressed and False otherwise.'''
        return not self.right_limit_switch.is_pressed() and not self.left_limit_switch.is_pressed()
    



    # TODO: FIX LATER TO INCORPORATE CONCURRENCY
    def capture_object(self) -> None:
        '''Opens the claw and slowly closes it until an object is detected or the closing boundary is reached.
        
        This function opens the claw completely and then gradually closes it while
        checking for the presence of an object to capture. If an object is detected, 
        it stops closing the claw and returns. This function is designed to be run in a
        loop until an object is captured or the closing boundary is reached.'''

        #open the claw (start at 100%)
        open_percent = 100
        #increment down, slowly closing the claw
        while(open_percent >= 0):
            #Set new claw state
            self.set_percent_open(open_percent)
            #if object is detected to be captured, then return to execute next task
            if self.is_object_captured():
                break
            #sleep for short amount of time and decrease percent openess
            time.sleep(0.025)
            open_percent -= 10




    def release_object(self) -> None:
        '''Releases an object held by the claw by gradually opening it until the claw is fully open.
        
        This function starts with the claw at its current state and gradually opens it until
        it is fully open while periodically checking if the object has been fully released.
        If the object is fully released, the function stops opening the claw and returns.'''

        #start with the claw at its current state
        open_percent = self.percent_open
        #increment down, slowly closing the claw
        while(open_percent <= 100):
            #Set new claw state
            self.set_percent_open(open_percent)
            #if object is detected to be captured, then return to execute next task
            if self.is_object_fully_released():
                break
            #sleep for short amount of time and increment percent openess
            time.sleep(0.01)
            open_percent += 5