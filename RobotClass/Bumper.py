from Manipulation import LimitSwitch
#Note use sphero drive tank to control movement
#Info: https://github.com/sphero-inc/sphero-sdk-raspberrypi-python/blob/master/getting_started/observer/driving/drive_tank_si.py

class Bumper():
    def __init__(self, rvr):
        self.rvr = rvr
        self.left_bumper = LimitSwitch(pin=xxxxxxx, use_pullup_config=True)
        self.right_bumper = LimitSwitch(pin=xxxxxxx, use_pullup_config=True)
        self.leftBack_bumper = LimitSwitch(pin=xxxxxxx, use_pullup_config=True)
        self.rightBack_bumper = LimitSwitch(pin=xxxxxxx, use_pullup_config=True)

    def check_limit_pressed(self):
        #Robot moves and checks if limit switched is pressed
        if self.left_bumper.is_pressed:
            self._left_bumper_pressed()
        if self.right_bumper.is_pressed:
            self._right_bumper_pressed()
        if self.leftBack_bumper.is_pressed:
            self._leftBack_bumper_pressed()
        if self.rightBack_bumper.is_pressed:
            self._rightBack_bumper_pressed()


    def _left_bumper_pressed(self):
        #Determines movement when left bumper hit
        #As of now it spins until its not pressed but TBD
        #I did not run so 100% conceptual
        while self.left_bumper.is_pressed:
            self.rvr.drive_tank_si_units(
                left_velocity = 0.5,
                right_velocity = 0.1
            )

        self.rvr.time.sleep(1)
        self.rvr.drive_tank_si_unit(
            left_velocity = 0.5,
            right_velocity = 0.5
        )



    def _right_bumper_pressed(self):
        #Determines movement when right bumper hit
        while self.right_bumper.is_pressed:
            self.rvr.drive_tank_si_units(
                left_velocity = 0.1,
                right_velocity = 0.5
            )
            
        self.rvr.time.sleep(1)
        self.rvr.drive_tank_si_unit(
            left_velocity = 0.5,
            right_velocity = 0.5
        )


    def _leftBack_bumper_pressed(self):
        #Determines movement when left back bumper hit
        while self.left_bumper.is_pressed:
            self.rvr.drive_tank_si_units(
                left_velocity = 0.5,
                right_velocity = 0.1
            )

        self.rvr.time.sleep(1)
        self.rvr.drive_tank_si_unit(
            left_velocity = 0.5,
            right_velocity = 0.5
        )

    def _rightBack_bumper_pressed(self):
        #Determines movement when right back bumper hit
        while self.right_bumper.is_pressed:
            self.rvr.drive_tank_si_units(
                left_velocity = 0.1,
                right_velocity = 0.5
            )
            
        self.rvr.time.sleep(1)
        self.rvr.drive_tank_si_unit(
            left_velocity = 0.5,
            right_velocity = 0.5
        )
