#!/usr/bin/env python
import numpy as np
import rospy
from sensor_msgs.msg import Image
import cv2 as cv
import rospkg
import roslaunch
import tf2_ros

import matplotlib.pyplot as plt

from paths.trajectories import LinearTrajectory
from paths.path_planner import PathPlanner
from paths.paths import MotionPath


from trac_ik_python.trac_ik import IK

import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys
from intera_interface import gripper as robot_gripper

from trac_ik_python.trac_ik import IK

import rospy
import tf2_ros
import intera_interface
from moveit_msgs.msg import DisplayTrajectory, RobotState
from sawyer_pykdl import sawyer_kinematics

import os

import moveit_commander

PIXELS_TO_METERS = 515.
display_holes = False
start_pixels = (124.5, 279.) # x
start_position = (0.653, -0.372, -0.091)

# converting pixels to meters
def pixels_to_meters(pixels):
    return np.divide(pixels, PIXELS_TO_METERS)

# figure out where to go 
def move_to_ball(start_pixels, start_position, ball_position):
    x_pixels = start_pixels[0] - ball_position[0]
    y_pixels = ball_position[1] - start_pixels[1]
    x_meters = x_pixels / PIXELS_TO_METERS
    y_meters = y_pixels / PIXELS_TO_METERS
    return (start_position[0] + x_meters, start_position[1] + y_meters, start_position[2])

# tucks to position for reading image from the camera
def camera_tuck():
    """
    Tuck the robot arm to the start position. Use with caution
    """
    if input('Would you like to tuck the arm? (y/n): ') == 'y':
        rospack = rospkg.RosPack()
        path = rospack.get_path('putt_putt')
        launch_path = path + '/launch/camera_tuck.launch'
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_path])
        launch.start()
    else:
        print('Canceled. Not tucking the arm.')

curr_img = None
def callback_img(msg):
	global curr_img
	curr_img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)

joint_angles = None
def callback_joints(msg):
     global joint_angles
     joint_angles = msg.position[1:-1]
	
def get_scan():
    rospy.sleep(5)
    
    while not rospy.is_shutdown():
         if curr_img is not None:
              return curr_img
            # curr_img = curr_img.squeeze()
            # plt.imshow(curr_img, cmap='gray')
            # plt.show()
       
def get_circle_position(img, min_size, max_size):         
    img = curr_img
    circles = cv.HoughCircles(img, cv.HOUGH_GRADIENT, dp=1, minDist=20, param1=50, param2=20, minRadius= min_size, maxRadius= max_size)

    if circles is not None:
        return circles[0][0] # most confident center and radius
    else:
        print("no circles found")


if __name__ == '__main__':
     verbose = False
     if len(sys.argv) > 1:
         if sys.argv[1] == 'v':
             verbose = True
     


     # set up ros nodes 
     rospy.wait_for_service('compute_ik')
     rospy.init_node('ball_detection', anonymous = True)

     compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
     right_gripper = robot_gripper.Gripper('right_gripper')
     img_curr_msg = rospy.Subscriber('/io/internal_camera/right_hand_camera/image_raw', Image, callback_img, queue_size = 1)
     rate = rospy.Rate(.5)
     
     # go to scanning position
     camera_tuck() 
     # get a scan
     
     my_image = get_scan()
     
     # calculate desired position based on image
     ball_pos = get_circle_position(my_image, 0, 12)
     if verbose:
        print("Ball Position", ball_pos)

     hole_pos = get_circle_position(my_image, 17, 30)
     if verbose:
         print("Hole Position", hole_pos)

     # display findings
     if verbose:
         for circle in [ball_pos, hole_pos]:
             cv.circle(my_image, (int(circle[0]), int(circle[1])), int(circle[2]), color = (0, 0, 255))
         my_image = my_image.squeeze()
         plt.imshow(my_image, cmap='gray')
         plt.show()

     # move to swing position
     if not rospy.is_shutdown(): # instead of while because i'm sick of it running multiple times 
         input('Press [ Enter ] to swinging stuff: ')
        
         # Construct the request
         request = GetPositionIKRequest()
         request.ik_request.group_name = "right_arm"

         # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
         link = "right_gripper_tip"

         request.ik_request.ik_link_name = link
         # request.ik_request.attempts = 20
         request.ik_request.pose_stamped.header.frame_id = "base"


         joint_getter = rospy.Subscriber('/robot/joint_states', JointState, callback_joints, queue_size = 1)
        # xyz_getter = rospy.Subscriber('')

         def move_to(x, y, z, speed = .25):
            # Set the desired orientation for the end effector HERE
            request.ik_request.pose_stamped.pose.position.x = x
            request.ik_request.pose_stamped.pose.position.y = y
            request.ik_request.pose_stamped.pose.position.z = z     
            request.ik_request.pose_stamped.pose.orientation.x = 0.
            request.ik_request.pose_stamped.pose.orientation.y = 1.
            request.ik_request.pose_stamped.pose.orientation.z = 0.
            request.ik_request.pose_stamped.pose.orientation.w = 0.
            
            try:
                # Send the request to the service
                response = compute_ik(request)
                
                # Print the response HERE
                #  print("response", response)
                group = MoveGroupCommander("right_arm")
                # group.limit_max_cartesian_link_speed(speed)
                # group.set_max_velocity_scaling_factor(1.0) 
                # Setting position and orientation target
                group.set_pose_target(request.ik_request.pose_stamped)

                # TRY THIS
                # Setting just the position without specifying the orientation
                # if speed != 0.1:
                #     group.set_position_target([0.5, 0.5, 0.0])

                # Plan IK
                plan = group.plan()
                # my_joints = None
                # while my_joints is None:
                #     my_joints = joint_angles
                # print(my_joints)
                # print("robot commande:", moveit_commander.RobotCommander().get_current_state())
                plan = group.retime_trajectory(moveit_commander.RobotCommander().get_current_state(), plan[1], velocity_scaling_factor=speed)
                # plan = group.retime_trajectory(group.get_current_state(), plan, velocity_scaling_factor=1)
                # plan = group.retime_trajectory(my_joints, plan, velocity_scaling_factor=1)
                # print(plan)
                user_input = input("Enter 'y' if the trajectory looks safe on RVIZ: ")
                
                # Execute IK if safe
                if user_input == 'y':
                    print("yuh let's go")
                    # group.go(wait=True)
                    group.execute(plan)
                    return True 
                return False
                
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return False

         
        # # go to setup position
         os.system(f"rosrun intera_examples go_to_joint_angles.py -s 0.2 -q -1.4944794921875 0.5608037109375 -1.655158203125 -1.43123828125 0.968298828125 1.3673369140625 1.766833984375")   
         
         swing_start = move_to_ball(start_pixels, start_position, ball_pos)
         accepted_move = False
         while not accepted_move:
             accepted_move = move_to(swing_start[0], swing_start[1], swing_start[2] + .02) 
             

         
         y = hole_pos[1] - ball_pos[1]
         x = hole_pos[0] - ball_pos[0]
         theta = np.arctan(float(y / x))

         

         back_swing_x = swing_start[0] + .1 * np.cos(theta)
         back_swing_y = swing_start[1] + .1 * np.sin(theta)
         back_swing_z = swing_start[2] - .04

         back_position = np.array([back_swing_x, back_swing_y, back_swing_z])

        #  print("curr xyz pos: ", swing_start)
        #  print("swing x: ", back_swing_x)
        #  print("swing y: ", back_swing_y)
        #  print("swing z: ", back_swing_z)
         rospy.sleep(2)
         my_joints = None
         while my_joints is None:
             my_joints = joint_angles
        

         os.system(f"rosrun intera_examples go_to_joint_angles.py -q {my_joints[0]} {my_joints[1]} {my_joints[2]} {my_joints[3]} {my_joints[4] } {my_joints[5]} {my_joints[6] + theta}")   


         
         accepted_move = False
         while not accepted_move:
             accepted_move = move_to(back_swing_x, back_swing_y, swing_start[2] + .02)
        
         accepted_move = False
         while not accepted_move:
             accepted_move = move_to(back_swing_x, back_swing_y, back_swing_z)


### FRONT SWING 
         front_swing_x = swing_start[0] - .12 * np.cos(theta)
         front_swing_y = swing_start[1] - .12 * np.sin(theta)
         front_swing_z = swing_start[2] - .04
         
         front_position = np.array([front_swing_x, front_swing_y, front_swing_z])

        #  accepted_move = False
        #  while not accepted_move:
        #      accepted_move = move_to(front_swing_x, front_swing_y, front_swing_z, speed=2.0)

# GO TOWARDS HOLE 

        #  hole_pos_meters = move_to_ball(ball_pos, back_position, hole_pos)
        #  accepted_move = False
        #  while not accepted_move:
        #      accepted_move = move_to(hole_pos_meters[0], hole_pos_meters[1], hole_pos_meters[2], speed = 0.1)

# ## LAB 7 FOR THE FINAL STRETCH! SO WE CAN SET SPEED
         tfBuffer = tf2_ros.Buffer()
         listener = tf2_ros.TransformListener(tfBuffer)

         try:
            trans = tfBuffer.lookup_transform('base', 'right_hand', rospy.Time(0), rospy.Duration(10.0))
         except Exception as e:
            print(e)
         current_position = np.array([getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')])
         print("Current Position:", current_position)

         ik_solver = IK("base", "right_gripper_tip")
         limb = intera_interface.Limb("right")
         kin = sawyer_kinematics("right")
         
         trajectory = LinearTrajectory(start_position=current_position, goal_position=front_position, total_time=2) # change time depending on desired speed
         trajectory.display_trajectory()
         path = MotionPath(limb, kin, ik_solver, trajectory)
         print("287")
         robot_trajectory = path.to_robot_trajectory(20, True) 

         planner = PathPlanner('right_arm')

         print("AFter")
    
         # By publishing the trajectory to the move_group/display_planned_path topic, you should 
        # be able to view it in RViz.  You will have to click the "loop animation" setting in 
        # the planned path section of MoveIt! in the menu on the left side of the screen.
         pub = rospy.Publisher('move_group/display_planned_path', DisplayTrajectory, queue_size=10)
         disp_traj = DisplayTrajectory()
         disp_traj.trajectory.append(robot_trajectory)
         disp_traj.trajectory_start = RobotState()
         pub.publish(disp_traj)
         user_input = input("Enter 'y' if the trajectory looks safe on RVIZ: ")
                
                # Execute IK if safe
         if user_input != 'y':
            raise Exception
    

         planner.execute_plan(robot_trajectory)
         
        # read xyz so we can use move it

        # rospy.sleep(1) 
        # if joint_angles is not None: # turn the gripper 
        #      my_joints = joint_angles
        #      print(my_joints)