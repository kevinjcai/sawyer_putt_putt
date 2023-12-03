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

curr_img = None

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
        print('Cancelled. Not tucking the arm.')

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


def get_circles_position(img, min_size, max_size):         
    img = curr_img
    circles = cv.HoughCircles(img, cv.HOUGH_GRADIENT, dp=1, minDist=20, param1=50, param2=20, minRadius= min_size, maxRadius= max_size)

    if circles is not None:
        return circles[0] # all fitting ones
    else:
        print("no circles found")

def callback_img(msg):
	global curr_img
	
	curr_img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
	
if __name__ == '__main__':

	# set up ros node 
	rospy.init_node('ball_detection', anonymous = True)
	img_curr_msg = rospy.Subscriber('/io/internal_camera/right_hand_camera/image_raw', Image, callback_img, queue_size = 1)
	rate = rospy.Rate(.5)
     
	# DISPLAY ONE
	rospy.sleep(2)
	while not rospy.is_shutdown():
          print("where my image at!")
          if curr_img is not None:
               my_image = curr_img
               found_circles = get_circles_position(my_image, 0, 12)
               for circle in found_circles:
                     cv.circle(my_image, (int(circle[0]), int(circle[1])), int(circle[2]), color = (0, 0, 255))
               my_image = my_image.squeeze()
               plt.imshow(my_image, cmap='gray')
               plt.show()
             
			# cv.imshow('image', curr_img)
			# cv.waitKey(0) 
			# cv.destroyAllWindows() 



	# get in position to read from camera
	# camera_tuck()

	# read scene from camera
	# only need one picture..... 

	## GOLF BALL DETECTION!

	# img = cv.imread("camera_setup.png", cv.IMREAD_GRAYSCALE)
	# print(img.shape)
	# # then run analysis! 
	# circles = cv.HoughCircles(img, cv.HOUGH_GRADIENT, dp=1, minDist=20, param1=50, param2=20, minRadius=0, maxRadius=10) # golf ball radius = approx 6; hole radius approx 13
    
	# print("found", len(circles[0]), "circles")
      
	# print(circles)
      
	# for circle in circles[0]:
	# 	cv.circle(img, (int(circle[0]), int(circle[1])), int(circle[2]), color = (0, 0, 255))
      
	# cv.imshow('image', img)
    
	# cv.waitKey(0) 
	# cv.destroyAllWindows() 


	# get hole position
	# get ball position 

	## SAVING PICS! 
	# img_counter = 0
	# while not rospy.is_shutdown():
	# 	if curr_img is not None:
	# 		img = curr_img
	# 		circles = cv.HoughCircles(img, cv.HOUGH_GRADIENT, dp=1, minDist=20, param1=50, param2=20, minRadius=15, maxRadius=30)

	# 		if circles is not None:
	# 			print(circles[0])

				

	# 			for circle in circles[0][:3]:
	# 				cv.circle(img, (int(circle[0]), int(circle[1])), int(circle[2]), color = (0, 0, 255))
				
	# 			img = img.squeeze()
	# 			plt.imshow(img, cmap='gray')
	# 			plt.show()
	# 			# cv.imwrite(f'pic{img_counter}.png', img)
	# 			img_counter += 1
	# 		else:
	# 			print("no circles found")

	# 	rate.sleep()



	# eventually publish the goal for movement or something

