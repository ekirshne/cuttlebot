from Robot import Robot
import time
import random

robot = Robot(
    #define the number of lives the cuttlebot has (number of times it can approach the predator before dieing)
    num_lives=3,
    
    #Define the parameters for the robot's learning and behavior
    learning_rate=0.2, 
    softmax_beta=1.0, 
    discount_factor=0.9, 

    #define the rewards for the predators and prey
    reward_predator=-10,
    reward_prey=5,
    
    #Define the number of predators and prey
    environment_type="HOMOGENEOUS", #{HOMOGENEOUS (Mixed), HETEROGENEOUS (Split), and PAIRED (Preditors & Prey close to eachother)} Change the Environment Type if you are changing the layout of the predators and prey
    num_predators=3,
    num_prey=3,

    #define the amount of time (in minutes to run the robot)
    runtime = 5 #minutes
)

robot.forage()


#==========================
#TEST CODE DOWN BELOW

#robot._align(35)
#robot._align(110)

#input_color = 110 #starting color
#while(input_color >= 0):
#    robot.camera_color_mask_view(input_color)
#    input_color = int(input("Hue Color: "))

'''
a = 1
while(1):
    print(a)
    current_time = time.time()
    while(time.time()-current_time < 5):
        if(random.random() < 0.5):
            robot.rvr.drive_tank_si_units(
                left_velocity = a*-0.1,
                right_velocity = a*-0.05
            )
        else:
            robot.rvr.drive_tank_si_units(
                left_velocity = a*0.05,
                right_velocity = a*0.1
            )
        time.sleep(0.25)

    current_time = time.time()
    while(time.time()-current_time < 5):
        if(random.random() < 0.5):
            robot.rvr.drive_tank_si_units(
                left_velocity = a*-0.05,
                right_velocity = a*-0.1
            )
        else:
            robot.rvr.drive_tank_si_units(
                left_velocity = a*0.1,
                right_velocity = a*0.05
            )
        time.sleep(0.25)
    a += 0.5
'''

#robot._test_shell()
#robot._run_blanch()


#print(robot.claw.get_percent_open())
#robot.claw.set_percent_open(100)
#time.sleep(2)
#print(robot.claw.get_percent_open())
#while(1):
#    time.sleep(0.5)


#robot.bumper.move_with_check(left_velocity=.3, right_velocity=0.3, distance_m=0.75)
#robot.bumper.move_with_check(left_velocity=.3, right_velocity=0.3, distance_m=-0.4)
#robot.rvr.drive_control.reset_heading()
#time.sleep(1)
#robot.rvr.drive_control.turn_left_degrees(
#            heading=0,  # Valid heading values are 0-359
#            amount=180
#        )
#time.sleep(1)
#robot.rvr.drive_control.reset_heading()
#time.sleep(1)
#robot.rvr.drive_control.turn_left_degrees(
#            heading=0,  # Valid heading values are 0-359
#            amount=180
#        )
#time.sleep(1)

#robot._align(110)

#robot._perform_action("APPROACH", 40)
#robot._perform_action("RUN", 40)

#robot._perform_action("APPROACH", 110)
#robot._perform_action("RUN", 110)

#while(1):
#    robot.bumper.move_with_check(0.3, 0.3, 5)
#    time.sleep(3)

#while(1):
#    robot._test_claw()

#while(True):
#    robot._run()

#robot._test_attack(0)

#robot._test_shell()
#print(robot._move_and_grab_color(0)) #0=red; 30=yellow
# robot._test_colision_detection()
#robot._test_shell()
#robot._test_env_movement()

#robot.do_nothing() #do nothing

#robot.test_move_and_grab()

#robot.camera_mode() #show robot camera POV
#robot.test_claw()
#robot.get_new_calibration_images(camera_ID=0) #take calibration images for the specified camera
#robot.get_object_depth_info_forward_to_back() #take pic, move forward, take pic, calculate distance
#robot.move_to_color() #follow a red object, maintaining a distance proportional to the size of the object
#robot.face_color(30) #0=red; 30=yellow

print("Done!")