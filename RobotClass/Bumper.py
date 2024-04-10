from Manipulation.LimitSwitch import LimitSwitch
import time
#Note use sphero drive tank to control movement
#Info: https://github.com/sphero-inc/sphero-sdk-raspberrypi-python/blob/master/getting_started/observer/driving/drive_tank_si.py

class Bumper():
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
                print("First")
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


"""from Manipulation.LimitSwitch import LimitSwitch
import time

class Bumper:
    def __init__(self, rvr, left_side_pin, right_side_pin, left_back_pin, right_back_pin):
        self.rvr = rvr
        # Initialize with GPIO pin numbers
        self.left_bumper = LimitSwitch(pin=left_side_pin, use_pullup_config=True)
        self.right_bumper = LimitSwitch(pin=right_side_pin, use_pullup_config=True)
        self.left_rear_bumper = LimitSwitch(pin=left_back_pin, use_pullup_config=True)
        self.right_rear_bumper = LimitSwitch(pin=right_back_pin, use_pullup_config=True)


    def check_limit_pressed(self):
        # Check each bumper and respond accordingly
        print("limitpressed?")
        while True:
            if self.left_bumper.is_pressed():
                print("leftbump")
                self.handle_bumper_press('left')
                print("leftbump2")
            if self.right_bumper.is_pressed():
                self.handle_bumper_press('right')
            if self.left_rear_bumper.is_pressed():
                self.handle_bumper_press('left_rear')
            if self.right_rear_bumper.is_pressed():
                self.handle_bumper_press('right_rear')


    def handle_bumper_press(self, bumper):
        # Handles movement based on which bumper is hit
        velocity_adjustment = {
            'left': (0.1, 0.5),
            'right': (0.5, 0.1),
            'left_rear': (0.1, 0.5),
            'right_rear': (0.5, 0.1)
        }
        left_velocity, right_velocity = velocity_adjustment[bumper]

        # Implement a safety timeout or condition to exit this loop if needed
        end_time = time.time() + 5  # 5 seconds from now
        self.rvr.drive_tank_si_units(
                left_velocity=left_velocity,
                right_velocity=right_velocity)
        while getattr(self, f"{bumper}_bumper").is_pressed() and time.time() < end_time:
            pass
        
        time.sleep(0.5)
        self.rvr.drive_tank_si_units(
            left_velocity=0.5, 
            right_velocity=0.5)"""