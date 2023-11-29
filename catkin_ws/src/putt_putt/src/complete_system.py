#!/usr/bin/env python
import numpy as np
import rospy
from sensor_msgs.msg import Image
import cv2 as cv
import rospkg
import roslaunch
import tf2_ros

import matplotlib.pyplot as plt


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

import os

PIXELS_TO_METERS = 515.
display_holes = False
start_pixels = (124.5, 279.) # x
start_position = (0.653, -0.372, -0.091)

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

# def lookup_tag(tag_number):
#     """
#     Given an AR tag number, this returns the position of the AR tag in the robot's base frame.
#     You can use either this function or try starting the scripts/tag_pub.py script.  More info
#     about that script is in that file.  

#     Parameters
#     ----------
#     tag_number : int

#     Returns
#     -------
#     3x' :obj:`numpy.ndarray`
#         tag position
#     """
#     tfBuffer = tf2_ros.Buffer()
#     tfListener = tf2_ros.TransformListener(tfBuffer)
#     try:
#         trans = tfBuffer.lookup_transform("base", f"ar_marker_{tag_number}", rospy.Time(), rospy.Duration(10.0))
#     except Exception as e:
#         print(e)
#         print("Retrying ...")

#     tag_pos = [getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')]
#     return np.array(tag_pos)

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
     ball_pos = get_circle_position(my_image, 0, 10)
     print(ball_pos)

     hole_pos = get_circle_position(my_image, 15, 30)
     print(hole_pos)

     # display findings
     if display_holes:
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

         def move_to(x, y, z):
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
                print(response)
                group = MoveGroupCommander("right_arm")

                # Setting position and orientation target
                group.set_pose_target(request.ik_request.pose_stamped)

                # TRY THIS
                # Setting just the position without specifying the orientation
                ###group.set_position_target([0.5, 0.5, 0.0])

                # Plan IK
                plan = group.plan()
                user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
                
                # Execute IK if safe
                if user_input == 'y':
                    group.execute(plan[1])
                
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

         

        # # go to setup position
         os.system(f"rosrun intera_examples go_to_joint_angles.py -s 0.2 -q -1.4944794921875 0.5608037109375 -1.655158203125 -1.43123828125 0.968298828125 1.3673369140625 1.766833984375")   
         
         swing_start = move_to_ball(start_pixels, start_position, ball_pos)
         move_to(swing_start[0], swing_start[1], swing_start[2])         
        
        # # read xyz so we can use move it

        # rospy.sleep(1) 
        # if joint_angles is not None: # turn the gripper 
        #      my_joints = joint_angles
        #      print(my_joints)