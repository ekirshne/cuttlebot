#The robot class (for now) will be composed of the RVR+, Vision module, and claw
import time
import numpy as np
from sphero_sdk import SpheroRvrObserver
from sphero_sdk import RvrStreamingServices
import cv2
import os
import random
from Manipulation.Claw import Claw
from Vision.Perception import Perception
from Cognition.Cognition import Cognition
from Bumper import Bumper
from Shell.Shell import Shell
import board

import colorsys
import datetime
from pathlib import Path
import csv

camouflage_color = np.array([0, 0, 0])
proportion_change = 0.15

def color_detected_handler(color_detected_data):
    global camouflage_color
    global proportion_change
    
    new_camouflage_color = (color_detected_data['ColorDetection']['R']/255.0, color_detected_data['ColorDetection']['G']/255.0, color_detected_data['ColorDetection']['B']/255.0)
    hsv = colorsys.rgb_to_hsv(*new_camouflage_color)
    
    print(hsv)

    modified_hsv = [
       hsv[0],
       0.25*hsv[0] + 0.75,
       hsv[2]
    ]
    modified_rgb = np.array(colorsys.hsv_to_rgb(*modified_hsv))
    
    read_camouflage_color = (255.0*modified_rgb).astype('i')

    camouflage_color = (1-proportion_change)*camouflage_color + proportion_change*read_camouflage_color
    camouflage_color = camouflage_color.astype('i')

    print(camouflage_color)
    

class Robot():

    #class constructor
    def __init__(self, num_lives, learning_rate, softmax_beta, discount_factor, reward_predator, reward_prey, environment_type, num_predators, num_prey, runtime):
        #save the number of lives that the cuttlebot has
        self.num_lives = num_lives
        
        #save the amount of time in minutes to run the forage program
        self.runtime = runtime #minutes

        #First, define the robot by its initialization time
        self.initialization_time = datetime.datetime.now()
        self.initialization_time_string = f"Y{self.initialization_time.year}_M{self.initialization_time.month}_D{self.initialization_time.day}_H{self.initialization_time.hour}_M{self.initialization_time.minute}_S{self.initialization_time.second}"
        
        #Get the experiment results folder
        self.experiments_data_path = Path(__file__).resolve().parent.joinpath("Experiment_Results")

        #Create folder to save data to
        self.run_data_path = self.experiments_data_path.joinpath(f"RobotRuns_{self.initialization_time_string}")
        self.run_data_path.mkdir(parents=True, exist_ok=False)

        #create a folder to save the images of interest to
        self.image_data_path = self.run_data_path.joinpath("Images")
        self.image_data_path.mkdir(parents=True, exist_ok=False)

        #Create a file of the actions taken by the robot
        self.actions_file_path = self.run_data_path.joinpath("Actions.csv")
        #update the Actions csv
        data = ["Timestamp", "Image_Path", "Object_Color", "Bounding_Box_X", "Bounding_Box_Y", "Bounding_Box_Width", "Bounding_Box_Height", "Action_Planned", "Action_Execution", "Reward"]
        self.write_header_to_action_csv(data)

        #create file with data of robot runs
        self.file_path = self.experiments_data_path.joinpath("RobotRuns.csv")
        if(self.file_path.is_file()):
            #if file exists, then only append data
            with open(self.file_path, 'a', newline='') as file:
                writer = csv.writer(file)
                data = [str(self.run_data_path), str(self.initialization_time.strftime("%Y-%m-%d %H:%M:%S")), str(self.num_lives), str(learning_rate), str(softmax_beta), str(discount_factor), str(reward_predator), str(reward_prey), str(environment_type), str(num_predators), str(num_prey), str(self.runtime)]
                writer.writerow(data)
        else:
            #if file doesnt exist, then create file and add headers
            with open(self.file_path, 'w', newline='') as file:
                writer = csv.writer(file)
                header = ["Folder", "Timestamp", "Num_Lives", "Learning_Rate", "Beta_Softmax", "Gamma_Discount_Factor", "Reward_Predator", "Reward_Prey", "Environment_Type", "Num_Predators", "Num_Prey", "Runtime_Minutes"]
                writer.writerow(header)
                data = [str(self.run_data_path), str(self.initialization_time.strftime("%Y-%m-%d %H:%M:%S")), str(self.num_lives), str(learning_rate), str(softmax_beta), str(discount_factor), str(reward_predator), str(reward_prey), str(environment_type), str(num_predators), str(num_prey), str(self.runtime)]
                writer.writerow(data)
        
        #Instantiate the modules on the robot alongside the robot itself
        #First the rvr
        self.rvr = SpheroRvrObserver()
        #give the rvr time to wake up
        self.rvr.wake()
        time.sleep(2)
        #Reset the YAW on the rvr
        self.rvr.reset_yaw()
        time.sleep(1)
        #reset the XY locator
        self.rvr.reset_locator_x_and_y()
        time.sleep(1)
        #Now the claw (pins 11 and 13 for the limit switches are not currently in use)
        self.claw = Claw(servo_pin=board.D17, right_limit_switch_pin=board.D27, left_limit_switch_pin=board.D22)
        #The vision module
        self.vision = Perception()
        
        #disable the Pan-tilt-unit due to faulty performance
        self.vision.pan_tilt_unit.set_servo_PWM_duty_cycles(0, 0)
        
        #The Cognition module
        #initiallize a color dictionary: assign a string to a hue value
        self.color_dict = {
            #"RED" : 175,
            "GREEN" : 35, # intended Predator
            "BLUE": 110 # intended Prey
        }
        #save the parameters of the robot
        self.learning_rate = learning_rate
        self.softmax_beta = softmax_beta
        self.discount_factor = discount_factor
        self.reward_predator = reward_predator
        self.reward_prey = reward_prey
        #Define the cognition module
        self.cognition = Cognition(self.color_dict, self.learning_rate, self.softmax_beta, self.discount_factor, data_path=self.run_data_path)
        #The Bumper
        self.bumper = Bumper(self.rvr, self.claw, left_side_pin=board.D24, right_side_pin=board.D16, left_back_pin=board.D25, right_back_pin=board.D20)
        #The Shell
        self.shell = Shell()
        self.shell.camouflage(color = (0,0,0), mode=1)
        time.sleep(1)

    def _test_claw(self):
        self.claw.set_percent_open(100)
        self.claw.set_percent_open(0)

    def _test_shell(self):
        self.rvr.enable_color_detection(is_enabled=True)
        self.rvr.sensor_control.add_sensor_data_handler(
            service=RvrStreamingServices.color_detection,
            handler=color_detected_handler
        )
        self.rvr.sensor_control.start(interval=250)
        while(1):
            #self.shell.hypnotize()
            #time.sleep(5)

            #get the color mask
            #mask = self.vision.camera.get_color_mask()

            # camouflage 
            # How to work with RVR's color detection: https://sdk.sphero.com/raspberry-pi-setup/how-to-use-raspberry-pi-sdk#h.vkbuw821vfgr
            self.shell.camouflage(color = camouflage_color, mode=1)
            #time.sleep(3)
            #self.rvr.enable_color_detection(is_enabled=False)
            #time.sleep(3)

            #If there are less than 20 active pixels
            # if np.sum(mask/255) < 50:
            #     #wait and try again
            #     self.shell.camouflage(color = (0, 255, 255), mode=1)
            #     self.vision.pan_tilt_unit.set_servo_angles(pan_angle=10, tilt_angle=10)
            # else:
            #     self.shell.camouflage(color = (255, 0, 0), mode=1)
            #     self.vision.pan_tilt_unit.set_servo_angles(pan_angle=-10, tilt_angle=-10)
            #wait to loop again
            time.sleep(0.25)
                

    def _test_env_movement(self):
        self.claw.set_percent_open(100)
        self.bumper.move_with_check(left_velocity=0.3, right_velocity=0.3, distance_m=5)
        print("All Done!")


    def _test_colision_detection(self):
        '''Test bumper switch'''
        self.bumper.check_limit_pressed()
        # self.bumper.check_limit_pressed()

    def _explore(self) -> None:
        '''Randomly turns the robot to -90, 0, or 90 degrees.
        
        This function randomly selects an angle from the set {-90, 0, 90} and commands 
        the robot to turn to that angle using the drive_to_position_si method of the 
        RVR object. It then pauses execution for 5 seconds to allow the robot to turn.'''

        self._explore_turn()
        
        time.sleep(1)
        self._explore_move

    def _explore_turn(self) -> None:
        random_turn = int(random.uniform(-179, 179))
        self.rvr.drive_control.turn_left_degrees(
            heading=0,  # Valid heading values are 0-359
            amount=random_turn
        )

    def _explore_move(self) -> None:
        random_move = random.uniform(-0.5, 0.5)
        self.bumper.move_with_check(
            left_velocity=0.3,
            right_velocity=0.3,
            distance_m=random_move
        )
        
    def _run_blanch(self) -> None:
        global proportion_change
        prev_proportion_change = proportion_change
        proportion_change = 0.35

        self.rvr.enable_color_detection(is_enabled=True)
        self.rvr.sensor_control.add_sensor_data_handler(
            service=RvrStreamingServices.color_detection,
            handler=color_detected_handler
        )
        self.rvr.sensor_control.start(interval=250)

        self.rvr.drive_tank_si_units(
            left_velocity = -0.5,
            right_velocity = -0.5
        )
        
        self.shell.blanching()

        time_wait = 0.3
        current_time = time.time()
        while(time.time()-current_time < time_wait):
            if(self.bumper.check_limit_pressed()):
                break

        #Stop and then camouflage
        self.rvr.drive_tank_si_units(
            left_velocity = 0,
            right_velocity = 0
        )
        current_time = time.time()
        wait_time = 1 #s
        while(time.time()-current_time < wait_time):
            self.shell.camouflage(color = camouflage_color, mode=1)
            time.sleep(0.05)
        
        proportion_change = prev_proportion_change
        

    def _run_camo(self) -> None:
        '''Commands the robot to run forward for a short duration.
        
        This function drives the robot forward by setting both left and right wheel velocities 
        to -0.5 (assuming negative velocity corresponds to forward motion) using the 
        drive_tank_si_units method of the RVR object. After a brief period of 0.25 seconds, 
        it stops the robot by setting both velocities to 0.'''

        ####we should make it so that the shell object creates these threads
        self.rvr.enable_color_detection(is_enabled=True)
        self.rvr.sensor_control.add_sensor_data_handler(
            service=RvrStreamingServices.color_detection,
            handler=color_detected_handler
        )
        self.rvr.sensor_control.start(interval=250)
        
        self.rvr.drive_tank_si_units(
            left_velocity = -0.15,
            right_velocity = -0.15
        )

        #WE COULD MAYBE DO SOMETHING WHERE IT BACKS UP UNTIL THE PREDITOR IS NO LONGER IN RANGE OF ITS SIGHT (USE BB_Y VALUE)
        time_wait = 2.5
        current_time = time.time()
        while(time.time()-current_time < time_wait):
            self.shell.camouflage(color = camouflage_color, mode=1)
            if(self.bumper.check_limit_pressed()):
                self.rvr.drive_tank_si_units(
                    left_velocity = -0.15,
                    right_velocity = -0.15
                )

        #Stop and then turn around
        self.rvr.drive_tank_si_units(
            left_velocity = 0,
            right_velocity = 0
        )
        self.rvr.drive_control.reset_heading()
        time.sleep(0.5)
        self.rvr.drive_control.turn_left_degrees(
            heading=0,  # Valid heading values are 0-359
            amount=180
        )

        start_time = time.time()
        while(time.time()-start_time < 1):
            self.shell.camouflage(color = camouflage_color, mode=1)
            time.sleep(0.1)
        
        self.rvr.drive_tank_si_units(
            left_velocity = 0.15,
            right_velocity = 0.15
        )

        start_time = time.time()
        while(time.time()-start_time < 1):
            self.shell.camouflage(color = camouflage_color, mode=1)
            time.sleep(0.1)

        self.rvr.drive_tank_si_units(
            left_velocity = 0,
            right_velocity = 0
        )

        self.rvr.enable_color_detection(is_enabled=False)
        self.rvr.sensor_control.stop()
        time.sleep(0.5)
        self.shell.turn_off_shell()


    def _align(self, color: int, timeout: float = 10.0) -> bool:
        '''Aligns the robot with a ball of the specified color.

        This function uses computer vision to detect and align the robot with a ball of the specified color. 
        It sets the color filter on the camera, then continuously adjusts the robot's position based on the 
        ball's position relative to the center of the camera's field of view.'''

        self.vision.camera.set_color_filter(color, precision=10)
        first_run = True
        stop = False
        K_p = 0.15
        K_i = 0.025
        robot_proportion_angle_deg = 0
        robot_sum_angle_deg = 0
        pan_offset = -5.0
        start_time = time.time()
        while(time.time()-start_time <= timeout):
            #shell hypnosis change time will act as delay for loop
            self.shell.next_hypnosis_step()

            mask = self.vision.camera.get_color_mask()
            #If there are less than 20 active pixels
            if np.sum(mask/255) < 20:
                time.sleep(0.1)
                if stop:
                    #lost track of object, exit loops
                    break
                stop = True
                continue

            #found object, reset stopping condition
            stop = False
            #The mask exists and we know we have found an objects (>=10 active pixels in mask)
            #Get center index in (Row,Col) format
            mask_center = np.flip((np.array(mask.shape)-1)/2)
            #avg_point = self.vision.get_avg_mask_point(mask, relative_point=mask_center)
            
            #Get the largest contour for the mask and find the centroid relative to the center of the mask
            largest_contour = self.vision.get_largest_contour(mask)
            largest_contour_bounding_box = self.vision.get_contour_bounding_box(largest_contour)
            largest_contour_centroid = self.vision.get_contour_bounding_box_centroid(largest_contour_bounding_box)
            rel_point = self.vision.get_relative_position(largest_contour_centroid, relative_point=mask_center)

            #Now update the pan tilt unit according to the output of the control system (avg_point); with reference point at (0,0)
            #########self.vision.pan_tilt_unit.update(rel_point)
            #Now move the robot according to the current angle of the pan unit
            ##########cur_pan_angle = (self.vision.pan_tilt_unit.controller.PWM_duty_cycles[0]-7.5)/0.055556
            ##########cur_pan_angle -= pan_offset
            
            #TEMP FIX FOR FAULTY PAN-TILT-UNIT
            cur_pan_angle = -rel_point[0]/(self.vision.camera.width/2)
            cur_pan_angle *= 80.0 #160deg FOV -> +/- 80 deg

            #robot_proportion_angle_deg = K_p*cur_pan_angle
            #robot_sum_angle_deg += K_i*cur_pan_angle
            
            #compute the turn velocity
            #curr_left_velocity = -(robot_proportion_angle_deg+robot_sum_angle_deg)/90.0
            #curr_right_velocity = (robot_proportion_angle_deg+robot_sum_angle_deg)/90.0
            if(cur_pan_angle > 0):
                curr_left_velocity = -0.05
                curr_right_velocity = -0.025
            else:
                curr_left_velocity = -0.025
                curr_right_velocity = -0.05

            
            #Ensure that the velocity is never below 0.05 
            #if(curr_left_velocity > 0):
            #    curr_left_velocity = max(max(0.1,curr_left_velocity), 0.2)
            #    curr_right_velocity = min(min(-0.1,curr_right_velocity), -0.2)
            #else:
            #    curr_left_velocity = min(min(-0.1,curr_left_velocity), -0.2)
            #    curr_right_velocity = max(max(0.1,curr_right_velocity), 0.2)

            #check if the turn velocity has gotten small enough to stop update loop
            #print(np.abs(cur_pan_angle), np.abs(curr_left_velocity))

            if((not first_run) and (np.abs(cur_pan_angle+pan_offset) < 5.0)):
                self.rvr.drive_tank_si_units(
                    left_velocity = 0,
                    right_velocity = 0
                )
                print("STOP!")
                return True

            #if not, keep updating speed
            else:
                #have a small offset to move forward while flashing the shell
                #curr_left_velocity -= 0.05
                #curr_right_velocity -= 0.05
                alpha_proportion = 3.0 - 2.0*np.exp(-10.0*np.power((cur_pan_angle/80.0), 2))
                
                self.rvr.drive_tank_si_units(
                    left_velocity = alpha_proportion*curr_left_velocity,
                    right_velocity = alpha_proportion*curr_right_velocity
                )
            
            #can only reach stopping condition if not on first run
            first_run = False
            self.bumper.move_off_bumper_press()
            time.sleep(0.1)

        #timeout or lost track of object
        self.rvr.drive_tank_si_units(
            left_velocity = 0,
            right_velocity = 0
        )
        return False

    def _stalk(self, color: str) -> float: #float | None:
        '''Attempts to stalk an object of the specified color.

        This function attempts to track an object of the specified color. It sets the color 
        filter on the camera to detect the object and then takes multiple pictures to calculate 
        the average bounding box pixel width of the detected object. It then moves radially inward 
        by a predefined distance and repeats the process to calculate the object's depth. If the 
        object is not found or there is not enough difference in the widths of the object at 
        different positions, it returns None or -1, respectively.'''


        #reset cameras
        self.vision.pan_tilt_unit.set_servo_angles(0, 0)
        #Look for object
        self.vision.camera.set_color_filter(color, precision=10)
        #take 3 pictures and get the average bounding box pixel width of all detected one
        p_width_1 = 0
        img_found = 0
        for i in range(5):
            #shell hypnosis step acts as sleep due to large time to update
            self.shell.next_hypnosis_step()
            #get the color mask
            mask = self.vision.camera.get_color_mask()
            #If there are less than 20 active pixels
            if np.sum(mask/255) < 20:
                #wait and try again
                time.sleep(0.1)
                print("Width_1=None")
                continue
            #Get the largest contour for the mask and find the centroid relative to the center of the mask
            largest_contour = self.vision.get_largest_contour(mask)
            largest_contour_bounding_box = self.vision.get_contour_bounding_box(largest_contour)
            if largest_contour_bounding_box[2] < 5:
                print("Object too small!")
                print(f"width = {largest_contour_bounding_box[2]}")
                time.sleep(0.1)
                continue
            p_width_1 += largest_contour_bounding_box[2]
            print(f"Width_1={largest_contour_bounding_box[2]}")
            #The mask exists and we know we have found an objects (>=20 active pixels in mask)
            img_found += 1

        #get the average width or specify no width for no object being found
        if img_found != 0:
            print(f"total_width_1={p_width_1}")
            p_width_1 /= img_found
            print(f"img_found={img_found}")
            print(f"avg_width_1={p_width_1}")
        else:
            p_width_1 = None
            return -1
        #move radially inward by m meters
        movement = -0.15 #m
        self.shell.next_hypnosis_step()
        self.rvr.reset_yaw()
        time.sleep(0.1)

        self.shell.next_hypnosis_step()
        self.rvr.reset_locator_x_and_y()
        time.sleep(0.1)

        self.rvr.drive_to_position_si(
            yaw_angle = 0,
            x=0,
            y=movement,
            linear_speed=0.15,
            flags=0 if movement>0 else 1
        )

        start_time = time.time()
        while(time.time()-start_time < 1):
            self.shell.next_hypnosis_step()
            if(self.bumper.move_off_bumper_press()):
                break
            time.sleep(0.05)

        #get the average width of 3 pictures for the object at the new position
        p_width_2 = 0
        img_found = 0
        for i in range(5):
            #shell hypnosis step acts as sleep due to large time to update
            self.shell.next_hypnosis_step()
            #get the color mask
            mask = self.vision.camera.get_color_mask()
            #If there are less than 20 active pixels
            if np.sum(mask/255) < 20:
                #wait and try again
                time.sleep(0.1)
                print("Width_2=None")
                continue
            #Get the largest contour for the mask and find the centroid relative to the center of the mask
            largest_contour = self.vision.get_largest_contour(mask)
            largest_contour_bounding_box = self.vision.get_contour_bounding_box(largest_contour)
            if largest_contour_bounding_box[2] < 5:
                print("Object width too small!")
                print(f"width = {largest_contour_bounding_box[2]}")
                time.sleep(0.1)
                continue
            p_width_2 += largest_contour_bounding_box[2]
            print(f"Width_2={largest_contour_bounding_box[2]}")
            #The mask exists and we know we have found an objects (>=20 active pixels in mask)
            img_found += 1

        #get the average width or specify no width for no object being found
        self.shell.next_hypnosis_step()
        if img_found != 0:
            print(f"total_width_2={p_width_2}")
            p_width_2 /= img_found
            print(f"img_found={img_found}")
            print(f"avg_width_2={p_width_2}")
        else:
            p_width_2 = None
            return -1
        #Ensure that the width of the object was found in both cases
        if p_width_1==None or p_width_2==None:
            print(f"Images not found! width1={p_width_1}, width2={p_width_2}")
            return -1
        elif np.abs(p_width_1-p_width_2) < 2:
                print("WARNING: Not enough difference in images to properly compute depth info")
                return 0
        #Use the two average widths of the two positions to get an estimation of depth
        obj_depth = self.vision.get_temporal_difference_object_depth(
            p_length1=p_width_1, 
            p_length2=p_width_2, 
            distance_moved_radially_inward=movement
        )
        print(obj_depth)
        return obj_depth
    
    def _pounce(self, color: str, depth: float) -> float: #float | None:)
        #case after object

        #CAN MAKE POUNCE INTO A PROBABILISTIC EVENT (NEED TO SEARCH LITERATURE FIRST)

        #pounce offset is 0.2m for depths from 0-0.6m and x% from 0.6m and beyond
        #x determined to be 0.2m at 0.6m depth: x = 0.2/0.6 = 0.33
        #0.6m depth cutoff chosen since that is when the depth estimation starts to break down
        pounce_offset = max(0.2, 0.33*depth)  #m
        depth_proportion = depth-pounce_offset #only until roughly 0.2 meters away
        pounce_limit = pounce_offset #m
        if(depth > pounce_limit):
            velocity = 0.85
            self.rvr.drive_tank_si_units(
                left_velocity = velocity,
                right_velocity = velocity
            )
            time.sleep(depth_proportion/velocity)

        #transition into move and grab code
        reward = self._move_and_grab_color(color, timeout_sec=5)
        return(reward)

    def _approach(self, color: int) -> float:
        self.shell.start_hypnosis()

        is_aligned = self._align(color)
        #exit is not aligned
        if(not is_aligned):
            self.shell.turn_off_shell()
            print("action aborted")
            return(0)
        
        depth = self._stalk(color)
        #exit is stalk lost object
        if(depth < 0):
            self.shell.turn_off_shell()
            print("action aborted")
            return(0)
        
        print(f"depth = {depth}")

        #open up the claw
        self.claw.set_percent_open(100)
        time.sleep(0.1)
        
        self.shell.turn_off_shell()

        #now pounce and try to grab object
        reward = self._pounce(color, depth)

        #return the reward
        print(f"reward = {reward}")
        return(reward)

    def _test_attack(self, color: int) -> None:
        '''Tests the robot's attack mechanism against an object of the specified color.

        This function aligns the robot with the object of the specified color using the 
        _align method, then attempts to estimate the depth of the object using the _stalk 
        method. If the object is found and its depth is successfully estimated, the function 
        proceeds to execute the attack mechanism. It opens the claw, moves towards the object 
        at a predefined velocity, captures the object with the claw, and finally calculates 
        the reward received for the action.'''

        self._align(color)
        depth = self._stalk(color)
        print(f"depth = {depth}")
        if depth == None:
            print("FAIL!")
            return
        self.claw.set_percent_open(100)
        time.sleep(0.1)
        reward = self._pounce(color, depth)
        print(f"Reward = {reward}")

    def _get_blanching_probability(self, area_proportion):
        #error check
        if(area_proportion <= 0):
            return(0.0)
        #otherwise, compute proportion
        return(
            #equation based on softmax with a logarithmic x-axis modified s.t. p(5) = 0.5
            np.exp(-3.4657/(100.0*area_proportion))
        )
    
    def write_header_to_action_csv(self, header):
        #create file with the actions taken by the robot
        with open(self.actions_file_path, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(header)

    def append_data_to_action_csv(self, data):
        #create file with the actions taken by the robot
        with open(self.actions_file_path, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(data)


    def _perform_action(self, action: str, color: int) -> float:
        '''Performs the specified action.

        This function performs the specified action based on the provided action type. 
        If the action is 'APPROACH', it aligns the robot with the object of the specified 
        color, moves towards the object, attempts to grab it, releases the object, and then 
        moves backward to a safe distance. If the action is 'RUN', it commands the robot to 
        run away for a short duration. After performing the action, it returns the reward 
        received for the action.'''

        current_datetime = datetime.datetime.now()
        timestamp = current_datetime.strftime("%Y-%m-%d %H:%M:%S")
        #get the current view of the robot
        image = self.vision.camera.get_image()
        bounding_box = self.vision.get_bounding_box_for_color_in_image(image, color)
        if(bounding_box == None):
            bounding_box = (-1, -1, 0, 0)
            #Only save image, no bounding box needed
        else:
            #save the image with the bounding box around the object of interest
            image = cv2.rectangle(image, (bounding_box[0], bounding_box[1]), (bounding_box[0]+bounding_box[2], bounding_box[1]+bounding_box[3]), color=(255,0,0), thickness=2)
        
        #save the image to the image data directory
        datetime_string_name = f"Y{current_datetime.year}_M{current_datetime.month}_D{current_datetime.day}_H{current_datetime.hour}_M{current_datetime.minute}_S{current_datetime.second}"
        image_path = self.image_data_path.joinpath(f"image_{datetime_string_name}.png")
        image = cv2.flip(image, 0)
        cv2.imwrite(str(image_path), image)

        #Get the name of the color from the color dict
        color_name = None
        for name in self.color_dict.keys():
            if(self.color_dict[name] == color):
                color_name = name
                break
        
        #perform the action
        if action == 'APPROACH':
            reward = self._approach(color)
            
            self.claw.set_percent_open(100)

            if(reward < 0):
                #update the csv
                data = [str(timestamp), str(image_path), str(color_name), str(bounding_box[0]), str(bounding_box[1]), str(bounding_box[2]), str(bounding_box[3]), "APPROACH", "FAILED", reward]
                self.append_data_to_action_csv(data)
                #decrease the number of lives if the color was green
                if(color_name == "GREEN"):
                    self.num_lives -= 1
                #blanch and then camo
                self._run_blanch()
                self._run_camo()
            elif(reward == 0):
                #Item lost and action was abandoned
                #update the csv
                data = [str(timestamp), str(image_path), str(color_name), str(bounding_box[0]), str(bounding_box[1]), str(bounding_box[2]), str(bounding_box[3]), "APPROACH", "ABANDONED", reward]
                self.append_data_to_action_csv(data)
            else:
                #update the csv
                data = [str(timestamp), str(image_path), str(color_name), str(bounding_box[0]), str(bounding_box[1]), str(bounding_box[2]), str(bounding_box[3]), "APPROACH", "SUCCESS", reward]
                self.append_data_to_action_csv(data)

                #release object and move away
                self.rvr.drive_tank_si_units(
                    left_velocity = -0.1,
                    right_velocity = -0.1
                )
                time.sleep(1)
                self.rvr.drive_tank_si_units(
                    left_velocity = 0,
                    right_velocity = 0
                )
                self.rvr.drive_control.reset_heading()
                time.sleep(0.5)
                self.rvr.drive_control.turn_left_degrees(
                    heading=0,  # Valid heading values are 0-359
                    amount=180
                )
                time.sleep(1)
                self.rvr.drive_tank_si_units(
                    left_velocity = 0.2,
                    right_velocity = 0.2
                )
                time.sleep(1.5)
                self.rvr.drive_tank_si_units(
                    left_velocity = 0,
                    right_velocity = 0
                )
        #run away
        elif action == 'RUN':
            #Set the reward to be zero
            reward = 0
            #get the current view of the robot
            bb_area = bounding_box[2]*bounding_box[3]
            image_area = self.vision.camera.height*self.vision.camera.width
            bb_area_proportion = bb_area/image_area
            p_blanching = self._get_blanching_probability(bb_area_proportion)
            #test to see if the cuttlebot will blanch
            if(random.random() < p_blanching):
                #update the csv
                data = [str(timestamp), str(image_path), str(color_name), str(bounding_box[0]), str(bounding_box[1]), str(bounding_box[2]), str(bounding_box[3]), "RUN", "BLANCH", reward]
                self.append_data_to_action_csv(data)

                #blanch and then camo
                self._run_blanch()
                self._run_camo()
            #case for only camouflage
            else:
                #update the csv
                data = [str(timestamp), str(image_path), str(color_name), str(bounding_box[0]), str(bounding_box[1]), str(bounding_box[2]), str(bounding_box[3]), "RUN", "CAMOUFLAGE", reward]
                self.append_data_to_action_csv(data)
                #perform camooflauge action
                self._run_camo()
            time.sleep(0.5)
        #return to center and reset camera
        ###self.vision.pan_tilt_unit.set_servo_angles(0, 0)
        return reward

    #see color and perform action
    def forage(self):
        #loop forever and keep updating values
        #self._explore() # Rotate to a random direction
        counter = 1
        start_time = time.time()
        while((time.time() - start_time <= self.runtime*60.0)) and (self.num_lives > 0):
            #Get list of colors in view of the camera
            colors_in_view = self.vision.get_colors_in_view(self.color_dict)
            print(f"colors in view = {colors_in_view}")
            #Check if any colors were found
            if(len(colors_in_view) == 0):
                #explore: rotate to a random direction and see if any pictures were found again
                self._explore_turn()
                time.sleep(1)
                
                #Check for colors after turning the robot
                colors_in_view = self.vision.get_colors_in_view(self.color_dict)
                print(f"colors in view = {colors_in_view}")
                
                #Check if any colors were found
                if(len(colors_in_view) == 0):
                    #explore: rotate to a random direction and see if any pictures were found again
                    self._explore_move()
                    time.sleep(0.5)
                    #if there were still not object in view, then restart the loop
                    continue

            # Define the state of the robot: prioritize state with the most negative possible outcome (i.e. if the robot sees a prey and predator at the same time, it will prioritize to run away from the predator. But if the robot only sees one color in its view, it will always return that color as the state)
            #state = self.cognition.get_state_with_largest_view(colors_in_view)
            state = self.cognition.get_state_with_largest_punishment(colors_in_view)
            action = self.cognition.get_action(state)
            reward = self._perform_action(action, color=self.color_dict[state])
            new_state = self.cognition.get_new_state(state, action)
            self.cognition.update_q_table(state, action, reward, new_state)
            #save the current Q-table
            # if(counter % 5 == 0):
            #     print(f"Result After Trial #{counter}:")
            #     self.cognition.print_q_table()
            #update the counter
            counter += 1
            time.sleep(0.5)

    def _move_and_grab_color(self, color: int, timeout_sec: float=5) -> int:
        '''Moves the robot towards an object of the specified color and attempts to grab it.

        This function adjusts the robot's position based on the detected object's position and 
        attempts to move towards it while continuously aligning itself. Once the object is close 
        enough, it attempts to grab it using the robot's claw. If the object is successfully 
        grabbed, the function returns a reward of 5; otherwise, it returns a penalty of -10.'''

        #open the claw
        self.claw.set_percent_open(100)
        #Look for red object
        self.vision.camera.set_color_filter(color, precision=10)
        #if  color == 0:
        #    optimal_object_width = 100
        #else:
        #    optimal_object_width = 225
        #initiallize tank drive speed variabels
        driving_left_velocity = 0
        driving_right_velocity = 0

        #control loop
        current_time = time.time()
        object_lost = False
        while(time.time()-current_time < timeout_sec):
            # Get the color mask
            mask = self.vision.camera.get_color_mask()
            # If there are less than 10 active pixels
            if np.sum(mask/255) < 20:
                #stop the robot
                driving_left_velocity *= 0.75
                driving_right_velocity *= 0.75
                self.rvr.drive_tank_si_units(
                    left_velocity = driving_left_velocity,
                    right_velocity = driving_right_velocity
                )
                time.sleep(0.1)
                #If a double fail condition is hit (object is lost), exit out of loop and return a reward of 0
                if(object_lost):
                    break

                object_lost = True 
                continue
            
            #reset object lost condition
            object_lost = False

            #The mask exists and we know we have found an objects (>=100 active pixels in mask)
            #Get center index in (Row,Col) format
            mask_center = np.flip((np.array(mask.shape)-1)/2)
            #avg_point = self.vision.get_avg_mask_point(mask, relative_point=mask_center)
            #Get the largest contour for the mask and find the centroid relative to the center of the mask
            largest_contour = self.vision.get_largest_contour(mask)
            largest_contour_bounding_box = self.vision.get_contour_bounding_box(largest_contour)
            largest_contour_centroid = self.vision.get_contour_bounding_box_centroid(largest_contour_bounding_box)
            rel_point = self.vision.get_relative_position(largest_contour_centroid, relative_point=mask_center)
            
            #Now update the pan tilt unit according to the output of the control system (avg_point); with reference point at (0,0)
            ###########self.vision.pan_tilt_unit.update(np.array([rel_point[0], 0]))
            #Now move the robot according to the current angle of the pan unit
            ###########cur_pan_angle = (self.vision.pan_tilt_unit.controller.PWM_duty_cycles[0]-7.5)/0.055556 #!!!need to add servo class to PTU so we can get the servo angle

            #TEMP FIX FOR FAULTY PAN-TILT-UNIT
            cur_pan_angle = -rel_point[0]/(self.vision.camera.width/2)
            cur_pan_angle *= 80.0 #160deg FOV -> +/- 80 deg

            proportion = 0.25
            robot_proportion_angle_deg = proportion*cur_pan_angle
            #Filter out small robot movements
            if np.abs(robot_proportion_angle_deg) < 0.5:
                driving_left_velocity = 0
                driving_right_velocity = 0
            #if the movement is big enough, then move the robot
            else:
                driving_left_velocity = -robot_proportion_angle_deg/90.0
                driving_right_velocity = robot_proportion_angle_deg/90.0
            #in addition to turning the robot, add an offset to move it forward toward the object of intetest
            velocity_upper_limit = 0.3 #m/s
            velocity_lower_limit = 0.1 #m/s
            percentage_multiplier = 2.0
            bb_percent_to_bottom = largest_contour_bounding_box[1]/self.vision.camera.height
            driving_left_velocity += min(
                velocity_upper_limit,
                max(velocity_lower_limit, percentage_multiplier*bb_percent_to_bottom)
            )
            driving_right_velocity += min(
                velocity_upper_limit,
                max(velocity_lower_limit, percentage_multiplier*bb_percent_to_bottom)
            )
            #Offset to account for offcentering
            driving_left_velocity += 0.025
            
            #print("driving left velocity:", driving_left_velocity)
            #print("driving_right_velocity:", driving_right_velocity)
            #if( largest_contour_bounding_box[2] > optimal_object_width):
            if largest_contour_bounding_box[1]/self.vision.camera.height <= 0.01:
                #Change the velocity in proportion to the claw opennes
                driving_left_velocity *= self.claw.get_percent_open()/100.0
                driving_right_velocity *= self.claw.get_percent_open()/100.0

                # When the object is close enough, close the claw and stop the robot
                #self.claw.capture_object()
                if(self.claw.get_percent_open() >= 10 and not self.claw.is_object_captured()):
                    claw_open_percentage = self.claw.get_percent_open()
                    self.claw.set_percent_open(claw_open_percentage-10)
                    time.sleep(0.1)
                #claw is either closed all the way or has captured an object
                elif(self.claw.get_percent_open() < 10):
                    reward = self.reward_predator
                    return reward

                elif(self.claw.is_object_captured()):
                    #an object has been captured; shake and check if it is still active
                    self._shake()
                    #1 second time delay
                    time.sleep(1)
                    if(self.claw.is_object_captured()):
                        reward = self.reward_prey
                    else:
                        reward = self.reward_predator
                    return reward
                
                #error case, do not expect to run
                else:
                    reward = 0
                    return reward
            
            #after computing the new velocity of the wheels, set the values to the rvr
            self.rvr.drive_tank_si_units(
                left_velocity = driving_left_velocity,
                right_velocity = driving_right_velocity
            )

            # Show the camera view and the masked image
            #cv2.imshow("frame", self.vision.camera.get_image())
            #cv2.imshow("mask", mask)
        
        #return no reward if cannot get to object in given time
        reward = 0
        #stop the RVR if the object was lost or the action timed out
        self.rvr.drive_tank_si_units(
            left_velocity = 0,
            right_velocity = 0
        )
        return reward




    def do_nothing(self) -> None:
        '''Does nothing indefinitely.

        This function enters an infinite loop, effectively doing nothing until the program is terminated 
        externally or the loop is broken by some external condition.'''
        while True:
            pass




    def test_rvr_sensors(self) -> None:
        '''Tests the streaming functionality of RVR sensors.

        This method enables streaming of locator data from the RVR's sensors and starts 
        streaming at a specified rate. It then prints the enabled streaming services and 
        begins streaming data for a brief period. Finally, it stops streaming and disables 
        all streaming services.'''

        self.rvr.sensor_control.sensors["Locator"][0].enable_streaming_service("Locator")
        print(self.rvr.sensor_control.sensors["Locator"][0].enabled_streaming_services_by_id)
        rate_ms = 50 # minimum sample period is 33 ms
        self.rvr.sensor_control.start(rate_ms) #SOMTHING WRONG WITH START METHOD!
        print(self.rvr.sensor_control.enabled_sensors)
        print("=========================")
        #for i in range(50):
            #print(self.rvr.sensor_control.sensors["Locator"][0].streaming_services_configuration)
            #time.sleep(0.1)
        self.rvr.sensor_control.stop()
        self.rvr.sensor_control.sensors["Locator"][0].disable_all_streaming_services()
        print("Closed!")
    



    def _shake(self):
        '''Shakes the robot's claw to ensure a secure grip on the captured object.

        This method shakes the robot's claw in multiple directions to ensure that the 
        captured object is securely gripped. It alternates between moving the left and 
        right wheels of the robot in opposite directions to create a shaking motion.'''

        #start at mid
        velocity = 0.0
        delay = 0.01
        delta_v = 0.2

        #up
        for i in range(5):
            velocity += delta_v
            self.rvr.drive_tank_si_units(
                left_velocity = velocity,
                right_velocity = -velocity
            )
            time.sleep(delay)

        #down out
        for i in range(5):
            velocity -= delta_v
            self.rvr.drive_tank_si_units(
                left_velocity = velocity,
                right_velocity = -velocity
            )
            time.sleep(delay)

        #up
        for i in range(5):
            velocity += delta_v
            self.rvr.drive_tank_si_units(
                left_velocity = -velocity,
                right_velocity = velocity
            )
            time.sleep(delay)

        #down center
        for i in range(5):
            velocity -= delta_v
            self.rvr.drive_tank_si_units(
                left_velocity = -velocity,
                right_velocity = velocity
            )
            time.sleep(delay)

        #up
        for i in range(5):
            velocity += delta_v
            self.rvr.drive_tank_si_units(
                left_velocity = -velocity,
                right_velocity = velocity
            )
            time.sleep(delay)

        #down out
        for i in range(5):
            velocity -= delta_v
            self.rvr.drive_tank_si_units(
                left_velocity = -velocity,
                right_velocity = velocity
            )
            time.sleep(delay)

        #up
        for i in range(5):
            velocity += delta_v
            self.rvr.drive_tank_si_units(
                left_velocity = velocity,
                right_velocity = -velocity
            )
            time.sleep(delay)

        #down center
        for i in range(5):
            velocity -= delta_v
            self.rvr.drive_tank_si_units(
                left_velocity = velocity,
                right_velocity = -velocity
            )
            time.sleep(delay)
        
        #back up rapidly
        self.rapid_backup()




    def rapid_backup(self) -> None:
        '''Performs a rapid backward movement of the robot.

        This method commands the robot to rapidly move backward by setting the velocities of 
        both wheels to a high negative value. It then stops the backward movement after a 
        brief delay.'''

        velocity = 1.0
        delay = 0.15
        self.rvr.drive_tank_si_units(
            left_velocity = -velocity,
            right_velocity = -velocity
        )
        time.sleep(delay)
        self.rvr.drive_tank_si_units(
            left_velocity = 0,
            right_velocity = 0
        )




    def test_claw(self) -> None:
        '''Tests the functionality of the robot's claw.

        This method simulates the process of capturing, shaking, and releasing an object 
        using the robot's claw. It opens the claw, waits for a brief period, captures an 
        object, shakes the claw to ensure a secure grip, and then checks if the object is 
        successfully captured. If the object is captured, it releases the object and prints 
        a success message; otherwise, it prints a failure message.'''

        self.claw.set_percent_open(100)
        print("opening...")
        time.sleep(2)
        print("open")
        self.claw.capture_object()
        print("Object Captured!")
        time.sleep(1)
        self._shake()
        #self.rapid_backup()
        #1 second time delay
        time.sleep(1)
        if self.claw.is_object_captured():
            print("GOOD! :)")
            time.sleep(1)
            self.claw.release_object()
            print("Object Released!")
        else:
            print("BAD! D:")


    def camera_color_mask_view(self, color: int) -> None:
        print("Entering camera mode. Press ESC to quit")
        current_time = time.time()
        try:
            while(cv2.waitKey(100) != 27 and time.time()-current_time < 10):
                image = self.vision.camera.get_image()
                bounding_box = self.vision.get_bounding_box_for_color_in_image(image, color)
                if(bounding_box != None):
                    image = cv2.rectangle(image, (bounding_box[0], bounding_box[1]), (bounding_box[0]+bounding_box[2], bounding_box[1]+bounding_box[3]), (0, 255, 0), 2)
                    bb_area = bounding_box[2]*bounding_box[3]
                    print(f"bb_area = {bb_area}")
                    print(f"bb_width = {bounding_box[2]}")
                    print(f"bb_y = {bounding_box[1]}")
                    print(f"bb_area_proportion = {bb_area/(self.vision.camera.height*self.vision.camera.width)}")
                scaled_down_image = cv2.resize(image, (640, 480))
                cv2.imshow(f"Camera Image", scaled_down_image)
        except Exception as e:
            print(e)
            print("Stopping program")
        finally:
            #destroys all the windows we created before exiting the program
            cv2.destroyAllWindows()


    def camera_mode(self) -> None:
        '''Enters camera mode to display live camera feed.

        This method enters camera mode, where it continuously captures images from the robot's camera 
        and displays them on the screen. Pressing the ESC key exits camera mode and closes the display 
        window.'''

        print("Entering camera mode. Press ESC to quit")
        try:
            while cv2.waitKey(100) != 27:
                    img = self.vision.camera.get_calibrated_image(alpha=1.0)
                    #present the image in a scaled down form (to fit image on screen)
                    scaled_down_image = cv2.resize(img, (640, 480))
                    cv2.imshow(f"Camera Image", scaled_down_image)
            #destroys all the windows we created before exiting the program
            cv2.destroyAllWindows()
        #Incase of error or interrupt in loop, close all cv windows
        except:
            cv2.destroyAllWindows()




    def get_new_calibration_images(self, camera_ID: int) -> None:
        '''Allows capturing new calibration images for a specific camera.

        This method sets up a loop to continuously capture images from the specified camera and 
        save them as calibration images. Pressing the SPACE key saves the current image, while 
        pressing ESC exits the program. The captured images are saved to the directory specific 
        to the camera's ID in the camera calibration sets folder.'''

        #set up camera path to save images to
        print(f"Getting new calibration images for camera {camera_ID}...")
        file_directory_path = os.path.dirname(os.path.abspath(__file__))
        camera_directory_path = os.path.join(file_directory_path, "Vision")
        camera_directory_path = os.path.join(camera_directory_path, "Sight")
        camera_directory_path = os.path.join(camera_directory_path, "Camera_Calibration_Sets")
        camera_directory_path = os.path.join(camera_directory_path, f"Cam{camera_ID}")
        #initiallize image index
        img_index = 0
        #constantly loop until hitting ESC
        #press SPACE to save image
        print("Starting program...")
        print("Press SPACE to save image and ESC to exit")
        print("=========================")
        #get the first frame to set up the camera window
        window_identifier = f"Camera {camera_ID} Window"
        img = self.vision.camera.get_uncalibrated_image()
        #present the image in a scaled down form (to fit image on screen)
        scaled_down_image = cv2.resize(img, (640, 480))
        cv2.imshow(window_identifier, scaled_down_image)
        window_title = f"Camera: {camera_ID}; Image: {img_index}; Frame Countdown: 0"
        cv2.setWindowTitle(window_identifier, window_title)
        #set an optional frame timer to indicate when image will be taken
        frame_timer = 35
        frame_count = frame_timer
        while True:
            img = self.vision.camera.get_uncalibrated_image()
            #present the image in a scaled down form (to fit image on screen)
            scaled_down_image = cv2.resize(img, (640, 480))
            cv2.imshow(window_identifier, scaled_down_image)
            #change the title of the window
            window_title = f"Camera: {camera_ID}; Image: {img_index}; Frame Countdown: {frame_count}"
            cv2.setWindowTitle(window_identifier, window_title)
            #press SPACE to save the image
            if cv2.waitKey(1) == 32 or frame_count <= 0:
                #create image directory path
                camera_image_path = os.path.join(camera_directory_path, f"calibration_img{img_index}.png")
                #save image
                cv2.imwrite(camera_image_path, img)
                #increment image index
                img_index += 1
                #reset frame count
                frame_count = frame_timer
            #press ESC to exit
            if cv2.waitKey(1) == 27:
                #exit loop
                break
            #increase frame count
            frame_count -= 1
        #print the number of images saved
        print(f"{img_index} images saved to: {camera_directory_path}")
        #destroys all the windows we created before exiting the program
        cv2.destroyAllWindows()




    def __align_via_body_movement(self) -> None:
        '''Aligns the robot with an object using body movement.

        This method continuously adjusts the robot's movement based on the position of the 
        largest object detected by the camera. It uses proportional control to steer the robot 
        towards the object until it is aligned with it. Once aligned, the method stops the robot's movement.'''

        #initiallize tank drive speed variabels
        driving_left_velocity = 0
        driving_right_velocity = 0
        #control loop
        while True:
            #get the color mask
            mask = self.vision.camera.get_color_mask()
            #If there are less than 20 active pixels
            if np.sum(mask/255) < 20:
                #slow to a stop if nothing is found
                print("looking...")
                driving_left_velocity *= 0.95
                driving_right_velocity *= 0.95
                self.rvr.drive_tank_si_units(
                    left_velocity = driving_left_velocity,
                    right_velocity = driving_right_velocity
                )
                time.sleep(0.1)
                continue
            #The mask exists and we know we have found an objects (>=20 active pixels in mask)
            #Get center index in (Row,Col) format
            mask_center = np.flip((np.array(mask.shape)-1)/2)
            #avg_point = self.vision.get_avg_mask_point(mask, relative_point=mask_center)
            #Get the largest contour for the mask and find the centroid relative to the center of the mask
            largest_contour = self.vision.get_largest_contour(mask)
            largest_contour_bounding_box = self.vision.get_contour_bounding_box(largest_contour)
            largest_contour_centroid = self.vision.get_contour_bounding_box_centroid(largest_contour_bounding_box)
            rel_point = self.vision.get_relative_position(largest_contour_centroid, relative_point=mask_center)
            #Now move the robot proportional to the relative point of the largest object (P controller)
            max_speed = 0.25
            if np.abs(rel_point[0]) <= 1:
                #if object is centered, the return from procedure
                print("aligned!")
                driving_left_velocity = 0
                driving_right_velocity = 0
                self.rvr.drive_tank_si_units(
                    left_velocity = driving_left_velocity,
                    right_velocity = driving_right_velocity
                )
                return
            elif rel_point[0] > 0:
                #X-position of object to left of center -> turn right
                turning_speed = np.abs(rel_point[0]/self.vision.camera.width)*max_speed
                driving_left_velocity = turning_speed
                driving_right_velocity = -turning_speed
            else:
                #X-position of object to right of center -> turn left
                turning_speed = np.abs(rel_point[0]/self.vision.camera.width)*max_speed
                driving_left_velocity = -turning_speed
                driving_right_velocity = turning_speed
            #after computing the new velocity of the wheels, set the values to the rvr
            self.rvr.drive_tank_si_units(
                    left_velocity = driving_left_velocity,
                    right_velocity = driving_right_velocity
                )




    def align_with_object_at_distance(self) -> None:
        '''Aligns the robot with a red object at a specific distance.

        This method continuously adjusts the robot's movement based on the position and size 
        of the largest red object detected by the camera. It calculates the distance to the 
        object based on temporal differences in its size. It then adjusts the robot's velocity 
        to approach the object at a target distance.'''

        #Look for red object
        self.vision.camera.set_color_filter(0, precision=10)
        #initiallize tank drive speed variabels
        driving_left_velocity = 0
        driving_right_velocity = 0
        prev_width = None
        ###################prev_
        #reset IMU values
        self.rvr.reset_yaw()
        time.sleep(0.1)
        self.rvr.reset_locator_x_and_y()
        time.sleep(0.1)
        #control loop
        while True:
            #Give some waiting time before stating each loop
            time.sleep(0.1)
            #get the color mask
            mask = self.vision.camera.get_color_mask()
            #If there are less than 20 active pixels
            print(f"mask total active pixels = {np.sum(mask/255)}")
            if np.sum(mask/255) < 20:
                #stop the robot
                ########driving_left_velocity *= 0.75
                ########driving_right_velocity *= 0.75
                self.rvr.drive_tank_si_units(
                    left_velocity = driving_left_velocity,
                    right_velocity = driving_right_velocity
                )
                continue
            #The mask exists and we know we have found an objects (>=20 active pixels in mask)
            #Get center index in (Row,Col) format
            mask_center = np.flip((np.array(mask.shape)-1)/2)
            #Get the largest contour for the mask and find the centroid relative to the center of the mask
            largest_contour = self.vision.get_largest_contour(mask)
            largest_contour_bounding_box = self.vision.get_contour_bounding_box(largest_contour)
            p_width = largest_contour_bounding_box[2]
            #print(f"Width_1={largest_contour_bounding_box[2]}")
            #Initiallize the previous width value to compare
            if prev_width == None:
                p_width = prev_width
                movement = 0
                continue
            #get the depth from the previous and current width values
            obj_depth = self.vision.get_temporal_difference_object_depth(
                p_length1=prev_width, 
                p_length2=p_width, 
                distance_moved_radially_inward_m=movement
            )
            print(f"distance={obj_depth}")
            #open claw dependent on distance
            #
            #
            #
            #adjust velocity and move forward
            max_speed = 1 #m/s
            velocity_change = 0.2 #max_speed*(2/(1+np.exp(obj_depth/2)) - 1)
            driving_left_velocity = velocity_change
            driving_right_velocity = velocity_change
            self.rvr.drive_tank_si_units(
                left_velocity = driving_left_velocity,
                right_velocity = driving_right_velocity
            )

            
             


    def get_object_depth_info_forward_to_back(self) -> float: #float | None:
        '''Measures the depth of an object using the forward-to-backward approach.

        This method utilizes the robot's camera to detect and measure the depth of an object in front of it. 
        It first looks for a red object using the camera and aligns with it by adjusting the robot's position.
        Then, it takes multiple pictures of the object to measure its width. Afterward, the robot moves
        radially inward by a specified distance and re-aligns with the object. It takes more pictures of the
        object at the new position to measure its width again. Finally, the method calculates the depth of the
        object based on the change in width and movement distance.'''

        #Look for red object
        self.vision.camera.set_color_filter(0, precision=10)
        #face the object
        self.__align_via_body_movement()
        #take 3 pictures and get the average bounding box pixel width of all detected one
        p_width_1 = 0
        img_found = 0
        for i in range(5):
            #get the color mask
            mask = self.vision.camera.get_color_mask()
            #If there are less than 20 active pixels
            if np.sum(mask/255) < 20:
                #wait and try again
                time.sleep(0.1)
                print("Width_1=None")
                continue
            #The mask exists and we know we have found an objects (>=20 active pixels in mask)
            img_found += 1
            #Get the largest contour for the mask and find the centroid relative to the center of the mask
            largest_contour = self.vision.get_largest_contour(mask)
            largest_contour_bounding_box = self.vision.get_contour_bounding_box(largest_contour)
            p_width_1 += largest_contour_bounding_box[2]
            print(f"Width_1={largest_contour_bounding_box[2]}")
        #get the average width or specify no width for no object being found
        if img_found != 0:
            print(f"total_width_1={p_width_1}")
            p_width_1 /= img_found
            print(f"img_found={img_found}")
            print(f"avg_width_1={p_width_1}")
        else:
            p_width_1 = None
        #move radially inward by m meters
        movement = 0.1 #m
        self.rvr.reset_yaw()
        time.sleep(.1)
        self.rvr.reset_locator_x_and_y()
        time.sleep(.1)
        self.rvr.drive_to_position_si(
            yaw_angle = 0 if (movement>0) else 180,
            x=0,
            y=movement,
            linear_speed=0.25,
            flags=0
        )
        time.sleep(2.5)
        #align body once more with object
        self.__align_via_body_movement()
        #get the average width of 3 pictures for the object at the new position
        p_width_2 = 0
        img_found = 0
        for i in range(5):
            #get the color mask
            mask = self.vision.camera.get_color_mask()
            #If there are less than 20 active pixels
            if np.sum(mask/255) < 20:
                #wait and try again
                time.sleep(0.1)
                print("Width_2=None")
                continue
            #The mask exists and we know we have found an objects (>=20 active pixels in mask)
            img_found += 1
            #Get the largest contour for the mask and find the centroid relative to the center of the mask
            largest_contour = self.vision.get_largest_contour(mask)
            largest_contour_bounding_box = self.vision.get_contour_bounding_box(largest_contour)
            p_width_2 += largest_contour_bounding_box[2]
            print(f"Width_2={largest_contour_bounding_box[2]}")
        #get the average width or specify no width for no object being found
        if img_found != 0:
            print(f"total_width_2={p_width_2}")
            p_width_2 /= img_found
            print(f"img_found={img_found}")
            print(f"avg_width_2={p_width_2}")
        else:
            p_width_2 = None
        #Ensure that the width of the object was found in both cases
        if p_width_1==None or p_width_2==None:
            print(f"Images not found! width1={p_width_1}, width2={p_width_2}")
            return None
        #Use the two average widths of the two positions to get an estimation of depth
        obj_depth = self.vision.get_temporal_difference_object_depth(
            p_length1=p_width_1, 
            p_length2=p_width_2, 
            distance_moved_radially_inward=movement
        )
        print(obj_depth)
        return obj_depth




    def get_object_depth_info_side_to_side(self) -> float:
        '''Measures the depth of an object using the side-to-side approach.

        This method utilizes the robot's camera to detect and measure the depth of an object by moving
        from side to side relative to the object. It aligns with the largest object of interest in the camera's
        view and then turns the camera 90 degrees left. The robot also turns 90 degrees in the opposite direction.
        It then re-aligns with the object to ensure it's centered in the camera's view. After taking reference
        pictures, it moves forward and backward while taking pictures to compare the position of the object
        relative to the frame edges. Using trigonometry, it estimates the depth of the object from the camera's perspective.
        The depth is computed by comparing the object's position in the reference pictures and the pictures taken
        after moving forward and backward. The average depth estimate is then calculated and returned.'''


        #####Can also perform this task below for each contour in the image#####
        #Look for red object
        self.vision.camera.set_color_filter(0, precision=10)
        #set cameras to 0, 0 angle
        self.vision.pan_tilt_unit.set_servo_angles(0, 0)
        #face largest object with color of interest
        self.__align_via_body_movement()
        #turn camera 90deg either left or right (turning left right now, but can be dependent on environmental constraints)
        self.vision.pan_tilt_unit.set_servo_angles(90, 0)
        #turn RVR in opposite direction 90 degrees
        self.rvr.reset_yaw()
        time.sleep(.1)
        self.rvr.reset_locator_x_and_y()
        time.sleep(.1)
        self.rvr.drive_to_position_si(
            yaw_angle=-90,
            x=0,
            y=0,
            linear_speed=0.25,
            flags=0
        )
        time.sleep(2.5)
        #readjust rvr to center camera on largest object up
        self.__align_via_body_movement()
        #save centroid for reference
        print("mid SNAP!")###############
        #Move forward/back and compare the center-point of the object when the side first reaches the end of the frame OR when a limit of +/- 0.15m
        self.rvr.reset_yaw()
        time.sleep(.1)
        self.rvr.reset_locator_x_and_y()
        time.sleep(.1)
        self.rvr.drive_to_position_si(
            yaw_angle=0,
            x=0,
            y=0.15,
            linear_speed=0.25,
            flags=0
        )
        time.sleep(2)
        #take picture after moving forward 0.15m
        print("front SNAP!")################
        #move back 0.3m
        self.rvr.reset_yaw()
        time.sleep(.1)
        self.rvr.reset_locator_x_and_y()
        time.sleep(.1)
        self.rvr.drive_to_position_si(
            yaw_angle=0,
            x=0,
            y=0.15,
            linear_speed=0.25,
            flags=0
        )
        time.sleep(2)
        #take picture after being 0.15m back from reference position
        print("back SNAP!") ##############
        #use trig to estimate the depth information (2 results: mid w/ back & mid w/ front comparison)
        ##################################### depth_front_estimate = 
        ##################################### depth_back_estimate = 
        #take the average depth of the 2 computations
        depth = (depth_front_estimate + depth_back_estimate)/2
        #return the robot to the center, directly facing the object
        
        #print the estimated depth from the camera to the object
        print("Last statment!")




    def work_that_claw(self) -> None:
        '''Continuously opens and closes the claw in a loop.

        This method continuously opens the claw, waits for a specified duration, and then closes the claw
        for the same duration, creating a looping action of opening and closing the claw.'''

        while True:
            #open claw
            self.claw.set_percent_open(100)
            time.sleep(2.5)
            #close the claw
            self.claw.set_percent_open(0)
            time.sleep(2.5)

    
    

    def move_and_grab_red_ball(self) -> None:
        '''Looks for a red object, aligns the robot with it, moves towards it, and grabs it with the claw.

        This method continuously searches for a red object using the vision system, aligns the robot with the object,
        moves towards it, and grabs it with the claw. It stops the robot and opens the claw once the object is grabbed.'''

        #Look for red object
        self.vision.camera.set_color_filter(0, precision=10)
        #initiallize tank drive speed variabels
        driving_left_velocity = 0
        driving_right_velocity = 0

        is_claw_closed = False # Initialize the claw status
        #control loop
        while True:
            # Get the color mask
            mask = self.vision.camera.get_color_mask()
            # If there are less than 10 active pixels
            if np.sum(mask/255) < 20:
                #stop the robot
                driving_left_velocity *= 0.75
                driving_right_velocity *= 0.75
                self.rvr.drive_tank_si_units(
                    left_velocity = driving_left_velocity,
                    right_velocity = driving_right_velocity
                )
                time.sleep(0.1)
                continue
            #The mask exists and we know we have found an objects (>=20 active pixels in mask)
            #Get center index in (Row,Col) format
            mask_center = np.flip((np.array(mask.shape)-1)/2)
            #avg_point = self.vision.get_avg_mask_point(mask, relative_point=mask_center)
            #Get the largest contour for the mask and find the centroid relative to the center of the mask
            largest_contour = self.vision.get_largest_contour(mask)
            largest_contour_bounding_box = self.vision.get_contour_bounding_box(largest_contour)
            largest_contour_centroid = self.vision.get_contour_bounding_box_centroid(largest_contour_bounding_box)
            rel_point = self.vision.get_relative_position(largest_contour_centroid, relative_point=mask_center)
            
            #Now update the pan tilt unit according to the output of the control system (avg_point); with reference point at (0,0)
            #########self.vision.pan_tilt_unit.update(rel_point)
            #Now move the robot according to the current angle of the pan unit
            #########cur_pan_angle = (self.vision.pan_tilt_unit.controller.PWM_duty_cycles[0]-7.5)/0.055556 #!!!need to add servo class to PTU so we can get the servo angle

            #TEMP FIX FOR FAULTY PAN-TILT-UNIT
            cur_pan_angle = -rel_point[0]/(self.vision.camera.width/2)
            cur_pan_angle *= 80.0 #160deg FOV -> +/- 80 deg

            proportion = 0.3
            robot_proportion_angle_deg = proportion*cur_pan_angle
            #Filter out small robot movements
            if np.abs(robot_proportion_angle_deg) < 1.5:
                driving_left_velocity = 0
                driving_right_velocity = 0
            #if the movement is big enough, then move the robot
            else:
                driving_left_velocity = -robot_proportion_angle_deg/90.0
                driving_right_velocity = robot_proportion_angle_deg/90.0
            #in addition to turning the robot, add an offset to move it forward toward the object of intetest
            velocity_limit = 0.50 #m/s
            optimal_object_width = 200
            driving_left_velocity += velocity_limit*(1.0 - largest_contour_bounding_box[2]/optimal_object_width)
            driving_right_velocity += velocity_limit*(1.0 - largest_contour_bounding_box[2]/optimal_object_width)
            #after computing the new velocity of the wheels, set the values to the rvr
            self.rvr.drive_tank_si_units(
                    left_velocity = driving_left_velocity,
                    right_velocity = driving_right_velocity
                )
            
            print("driving left velocity:", driving_left_velocity)
            print("driving_right_velocity:", driving_right_velocity)
            if abs(driving_left_velocity) <= 0.1 and abs(driving_right_velocity) <= 0.1 and not is_claw_closed:
                # When the object is close enough, close the claw and stop the robot
                self.claw.capture_object()
                is_claw_closed = True
                self.rvr.drive_tank_si_units(
                    left_velocity = 0,
                    right_velocity = 0
                )

                # Open the claw after some time
                time.sleep(2)
                self.claw.release_object()
                break

            # Show the camera view and the masked image
            cv2.imshow("frame", self.vision.camera.get_image())
            cv2.imshow("mask", mask)




    def move_to_color(self) -> None:
        '''Moves the robot towards a detected color.

        This method continuously searches for a color using the vision system, aligns the robot with the detected color,
        and moves towards it. It adjusts both the direction and speed of the robot to ensure it moves smoothly towards the
        detected color.'''

        #Look for red object
        self.vision.camera.set_color_filter(0, precision=10)
        #initiallize tank drive speed variabels
        driving_left_velocity = 0
        driving_right_velocity = 0
        #control loop
        while True:
            #get the color mask
            mask = self.vision.camera.get_color_mask()
            #If there are less than 20 active pixels
            if np.sum(mask/255) < 20:
                #stop the robot
                driving_left_velocity *= 0.75
                driving_right_velocity *= 0.75
                self.rvr.drive_tank_si_units(
                    left_velocity = driving_left_velocity,
                    right_velocity = driving_right_velocity
                )
                time.sleep(0.1)
                continue
            #The mask exists and we know we have found an objects (>=20 active pixels in mask)
            #Get center index in (Row,Col) format
            mask_center = np.flip((np.array(mask.shape)-1)/2)
            #avg_point = self.vision.get_avg_mask_point(mask, relative_point=mask_center)
            #Get the largest contour for the mask and find the centroid relative to the center of the mask
            largest_contour = self.vision.get_largest_contour(mask)
            largest_contour_bounding_box = self.vision.get_contour_bounding_box(largest_contour)
            largest_contour_centroid = self.vision.get_contour_bounding_box_centroid(largest_contour_bounding_box)
            rel_point = self.vision.get_relative_position(largest_contour_centroid, relative_point=mask_center)
            
            #Now update the pan tilt unit according to the output of the control system (avg_point); with reference point at (0,0)
            ########self.vision.pan_tilt_unit.update(rel_point)
            #Now move the robot according to the current angle of the pan unit
            ########cur_pan_angle = (self.vision.pan_tilt_unit.controller.PWM_duty_cycles[0]-7.5)/0.055556 #!!!need to add servo class to PTU so we can get the servo angle

            #TEMP FIX FOR FAULTY PAN-TILT-UNIT
            cur_pan_angle = -rel_point[0]/(self.vision.camera.width/2)
            cur_pan_angle *= 80.0 #160deg FOV -> +/- 80 deg

            proportion = 0.3
            robot_proportion_angle_deg = proportion*cur_pan_angle
            #Filter out small robot movements
            if np.abs(robot_proportion_angle_deg) < 1.5:
                driving_left_velocity = 0
                driving_right_velocity = 0
            #if the movement is big enough, then move the robot
            else:
                driving_left_velocity = -robot_proportion_angle_deg/90.0
                driving_right_velocity = robot_proportion_angle_deg/90.0
            #in addition to turning the robot, add an offset to move it forward toward the object of intetest
            velocity_limit = 0.40 #m/s
            optimal_object_width = 150
            driving_left_velocity += velocity_limit*(1.0 - largest_contour_bounding_box[2]/optimal_object_width)
            driving_right_velocity += velocity_limit*(1.0 - largest_contour_bounding_box[2]/optimal_object_width)
            #after computing the new velocity of the wheels, set the values to the rvr
            self.rvr.drive_tank_si_units(
                    left_velocity = driving_left_velocity,
                    right_velocity = driving_right_velocity
                )




    def face_color(self, color: int =0) -> None:
        '''Faces the robot towards a detected color.

        This method continuously searches for the specified color using the vision system and aligns the robot's orientation
        with the detected color. It adjusts both the direction and speed of the robot to ensure it faces the color accurately.'''

        self.vision.camera.set_color_filter(color, precision=10)
        while True:
            mask = self.vision.camera.get_color_mask()
            #If there are less than 20 active pixels
            if np.sum(mask/255) < 20:
                time.sleep(0.1)
                continue
            #The mask exists and we know we have found an objects (>=10 active pixels in mask)
            #Get center index in (Row,Col) format
            mask_center = np.flip((np.array(mask.shape)-1)/2)
            #avg_point = self.vision.get_avg_mask_point(mask, relative_point=mask_center)
            #Get the largest contour for the mask and find the centroid relative to the center of the mask
            largest_contour = self.vision.get_largest_contour(mask)
            largest_contour_bounding_box = self.vision.get_contour_bounding_box(largest_contour)
            largest_contour_centroid = self.vision.get_contour_bounding_box_centroid(largest_contour_bounding_box)
            rel_point = self.vision.get_relative_position(largest_contour_centroid, relative_point=mask_center)
            
            #Now update the pan tilt unit according to the output of the control system (avg_point); with reference point at (0,0)
            ###############self.vision.pan_tilt_unit.update(rel_point)
            #Now move the robot according to the current angle of the pan unit
            ###############cur_pan_angle = (self.vision.pan_tilt_unit.controller.PWM_duty_cycles[0]-7.5)/0.055556

            #TEMP FIX FOR FAULTY PAN-TILT-UNIT
            cur_pan_angle = -rel_point[0]/(self.vision.camera.width/2)
            cur_pan_angle *= 80.0 #160deg FOV -> +/- 80 deg

            proportion = 0.4
            robot_proportion_angle_deg = proportion*cur_pan_angle
            #pan_compensation_PWM = 7.5 + 0.055556*(-robot_proportion_angle_deg)
            #new_pan_PWM = self.vision.pan_tilt_unit.controller.PWM_duty_cycles[0]+pan_compensation_PWM
            #self.vision.pan_tilt_unit.set_servo_PWM_duty_cycles(new_pan_PWM, self.vision.pan_tilt_unit.controller.PWM_duty_cycles[1])
            #self.rvr.reset_yaw()
            #self.rvr.reset_locator_x_and_y()
            #self.rvr.drive_to_position_si(
            #    linear_speed = 0.5,
            #    yaw_angle = robot_proportion_angle_deg,
            #    x = 0,
            #    y = 0,
            #    flags = 0
            #)

            #Filter out small robot movements
            if np.abs(robot_proportion_angle_deg) < 5.0:
                self.rvr.drive_tank_si_units(
                    left_velocity = 0,
                    right_velocity = 0
                )
            #if the movement is big enough, then move the robot
            else:
                self.rvr.drive_tank_si_units(
                    left_velocity = -robot_proportion_angle_deg/90.0,
                    right_velocity = robot_proportion_angle_deg/90.0
                )

    #class destructor
    def __del__(self) -> None:
        '''Class destructor method.

        This method is automatically called when the object is no longer in use and is being destroyed.
        It closes the connection to the RVR, ensuring proper cleanup.'''
        #Turn off the shell LEDs
        self.shell.camouflage(color = (0,0,0), mode=1)
        
        #close the rvr
        self.rvr.sensor_control.clear()

        # Delay to allow RVR issue command before closing
        time.sleep(.5)
        
        self.rvr.close()

    