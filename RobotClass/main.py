from Robot import Robot
import time

robot = Robot()

robot.forage()

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