from Manipulation.LimitSwitch import LimitSwitch
import time
from Manipulation.Claw import Claw

#Note use sphero drive tank to control movement
#Info: https://github.com/sphero-inc/sphero-sdk-raspberrypi-python/blob/master/getting_started/observer/driving/drive_tank_si.py

"""class Bumper():
    def __init__(self, rvr, left_side_pin, right_side_pin, left_back_pin, right_back_pin):
        self.rvr = rvr 
        self.left_bumper = LimitSwitch(pin=left_side_pin, use_pullup_config=True)       
        self.right_bumper = LimitSwitch(pin=right_side_pin, use_pullup_config=True)     
        self.leftBack_bumper = LimitSwitch(pin=left_back_pin, use_pullup_config=True)   
        self.rightBack_bumper = LimitSwitch(pin=right_back_pin, use_pullup_config=True) 
    
    def check_limit_switch(self):
        while True:
            print(self.left_bumper.is_pressed(), self.right_bumper.is_pressed(), self.leftBack_bumper.is_pressed(), self.rightBack_bumper.is_pressed())
            

    def check_limit_pressed(self):
        #Robot moves and checks if limit switched is pressed
        while True:
            if self.left_bumper.is_pressed():
                print("1")
                self._left_bumper_pressed()
            if self.right_bumper.is_pressed():
                print("2")
                self._right_bumper_pressed()
            if self.leftBack_bumper.is_pressed():
                print("3")
                self._leftBack_bumper_pressed()
            if self.rightBack_bumper.is_pressed():
                print("4")
                self._rightBack_bumper_pressed()
            time.sleep(0.1)

            



    def _left_bumper_pressed(self):
        #Determines movement when left bumper hit
        #As of now it spins until its not pressed but TBD
        #I did not run so 100% conceptual
        self.rvr.drive_tank_si_units(
                left_velocity = 0.5,
                right_velocity = 0.1
            )
        while self.left_bumper.is_pressed():
            time.sleep(0.1)
        
        time.sleep(1)
        self.rvr.drive_tank_si_units(
            left_velocity = 0.5,
            right_velocity = 0.5
        )



    def _right_bumper_pressed(self):
        #Determines movement when right bumper hit
        self.rvr.drive_tank_si_units(
                left_velocity = 0.1,
                right_velocity = 0.5
            )
        while self.right_bumper.is_pressed():
            time.sleep(0.1)
        
        time.sleep(1)
        self.rvr.drive_tank_si_units(
            left_velocity = 0.5,
            right_velocity = 0.5
        )


    def _leftBack_bumper_pressed(self):
        #Determines movement when left back bumper hit
        self.rvr.drive_tank_si_units(
                left_velocity = 0.5,
                right_velocity = 0.5
            )
        while self.leftBack_bumper.is_pressed():
            time.sleep(0.1)

        time.sleep(1)
        self.rvr.drive_tank_si_units(
            left_velocity = 0.5,
            right_velocity = 0.5
        )

    def _rightBack_bumper_pressed(self):
        #Determines movement when right back bumper hit
        self.rvr.drive_tank_si_units(
                left_velocity = 0.5,
                right_velocity = 0.5
            )
        while self.rightBack_bumper.is_pressed():
            time.sleep(0.1)
    
        time.sleep(1)
        self.rvr.drive_tank_si_units(
            left_velocity = 0.5,
            right_velocity = 0.5
        )
"""

from Manipulation.LimitSwitch import LimitSwitch
import time

class Bumper:
    def __init__(self, rvr, claw, left_side_pin, right_side_pin, left_back_pin, right_back_pin):
        self.rvr = rvr
        # Initialize with GPIO pin numbers
        self.claw = claw
        self.left_bumper = LimitSwitch(pin=left_side_pin, use_pullup_config=True)
        self.right_bumper = LimitSwitch(pin=right_side_pin, use_pullup_config=True)
        self.left_rear_bumper = LimitSwitch(pin=left_back_pin, use_pullup_config=True)
        self.right_rear_bumper = LimitSwitch(pin=right_back_pin, use_pullup_config=True)

    def move_with_check(self, left_velocity, right_velocity, distance_m):
        self.claw.set_percent_open(100)

        if(distance_m < 0):
            return(self.move_with_check(-left_velocity, -right_velocity, -distance_m))
        
        self.rvr.drive_tank_si_units(
            left_velocity=left_velocity, 
            right_velocity=right_velocity
        )
        
        avg_velocity = (left_velocity+right_velocity)/2.0
        time_to_move_distance = distance_m/avg_velocity
        current_time = time.time()
        while(time_to_move_distance > time.time()-current_time):
            time.sleep(0.1)
            was_pressed = self.check_limit_pressed()
            if(was_pressed):
                self.rvr.drive_tank_si_units(
                    left_velocity=left_velocity, 
                    right_velocity=right_velocity
                )
        #stop after moving approximate distance
        self.rvr.drive_tank_si_units(
            left_velocity=0, 
            right_velocity=0
        )


    def check_limit_pressed(self):
        # Check each bumper and respond accordingly
        if self.claw.right_limit_switch.is_pressed() or self.claw.left_limit_switch.is_pressed():
            self.handle_bumper_press('front')
        elif self.left_bumper.is_pressed():
            self.handle_bumper_press('left')
        elif self.right_bumper.is_pressed():
            self.handle_bumper_press('right')
        elif self.left_rear_bumper.is_pressed() or self.right_rear_bumper.is_pressed():
            self.handle_bumper_press('rear')
        else:
            return False
        #return true if limit switch was pressed during movement
        return True


    def handle_bumper_press(self, bumper):
        # Handles movement based on which bumper is hit
        # Note: only rear is using the velocity adjustments
        velocity_adjustment = {
            'front': (-0.25, -0.25),
            'left': (0.5, 0.1),
            'right': (0.1, 0.5),
            'rear': (0.25, 0.25),
        }
        left_velocity, right_velocity = velocity_adjustment[bumper]

        # Implement a safety timeout or condition to exit this loop if needed
        end_time = time.time() + 5  # 5 seconds from now
        
        #CHECK NEEDS TO BE FIXED TO BE A CLEANER
        if(bumper == "front"):
            self.rvr.drive_tank_si_units(
                left_velocity=left_velocity,
                right_velocity=right_velocity
            )
            while (self.claw.right_limit_switch.is_pressed() or self.claw.left_limit_switch.is_pressed()) and time.time() < end_time:
                time.sleep(0.1)
            #turn around after backing up a bit from the wall
            time.sleep(1)
            self.rvr.drive_control.reset_heading()
            self.rvr.drive_control.turn_right_degrees(
                heading=0,  # Valid heading values are 0-359
                amount=180
            )
            time.sleep(1)
        elif(bumper == "rear"):
            self.rvr.drive_tank_si_units(
                left_velocity=left_velocity,
                right_velocity=right_velocity
            )
            while self.left_rear_bumper.is_pressed() or self.right_rear_bumper.is_pressed():
                time.sleep(0.1)
            #give a second delay to move farther from wall
            time.sleep(1)
            #go straight forward
            self.rvr.drive_tank_si_units(
                left_velocity=0,
                right_velocity=0
            )
            time.sleep(1)
        elif(bumper == "left"):
            #turn right by 90 degrees
            self.rvr.drive_control.reset_heading()
            self.rvr.drive_control.turn_right_degrees(
                heading=0,  # Valid heading values are 0-359
                amount=90
            )
            time.sleep(1)
        elif(bumper == "right"):
            #turn left by 90 degrees
            self.rvr.drive_control.reset_heading()
            self.rvr.drive_control.turn_left_degrees(
                heading=0,  # Valid heading values are 0-359
                amount=90
            )
            time.sleep(1)
        else:
            while getattr(self, f"{bumper}_bumper").is_pressed() and time.time() < end_time:
                time.sleep(0.1)