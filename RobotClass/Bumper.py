from Manipulation.LimitSwitch import LimitSwitch
import time
#Note use sphero drive tank to control movement
#Info: https://github.com/sphero-inc/sphero-sdk-raspberrypi-python/blob/master/getting_started/observer/driving/drive_tank_si.py

class BumperClass():
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
            self.rvr.drive_tank_si_units(
                left_velocity = 0.5,
                right_velocity = 0.5
            )
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
        while self.left_bumper.is_pressed():
            self.rvr.drive_tank_si_units(
                left_velocity = 0.5,
                right_velocity = 0.1
            )
        
        time.sleep(1)
        
        return
        self.rvr.drive_tank_si_unit(
            left_velocity = 0.5,
            right_velocity = 0.5
        )



    def _right_bumper_pressed(self):
        #Determines movement when right bumper hit
        while self.right_bumper.is_pressed():
            self.rvr.drive_tank_si_units(
                left_velocity = 0.1,
                right_velocity = 0.5
            )
        
        time.sleep(1)
        return
        self.rvr.drive_tank_si_unit(
            left_velocity = 0.5,
            right_velocity = 0.5
        )


    def _leftBack_bumper_pressed(self):
        #Determines movement when left back bumper hit
        while self.left_bumper.is_pressed():
            self.rvr.drive_tank_si_units(
                left_velocity = 0.5,
                right_velocity = 0.1
            )
        time.sleep(1)
        return
        self.rvr.drive_tank_si_unit(
            left_velocity = 0.5,
            right_velocity = 0.5
        )

    def _rightBack_bumper_pressed(self):
        #Determines movement when right back bumper hit
        while self.right_bumper.is_pressed():
            self.rvr.drive_tank_si_units(
                left_velocity = 0.1,
                right_velocity = 0.5
            )
    
        time.sleep(1)
        return
        self.rvr.drive_tank_si_unit(
            left_velocity = 0.5,
            right_velocity = 0.5
        )
