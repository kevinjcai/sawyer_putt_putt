#!/usr/bin/env python
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

joint_angles = None

def callback_joints(msg):
     global joint_angles
     joint_angles = msg.position[1:-1]
     

def main():
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    # Set up the right gripper
    right_gripper = robot_gripper.Gripper('right_gripper')

    if not rospy.is_shutdown(): # instead of while because i'm sick of it running multiple times 
        input('Press [ Enter ]: ')
        
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
            # request.ik_request.pose_stamped.pose.orientation.x = 0.006
            # request.ik_request.pose_stamped.pose.orientation.y = 0.999
            # request.ik_request.pose_stamped.pose.orientation.z = -0.025
            # request.ik_request.pose_stamped.pose.orientation.w = 0.033

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

        # move_to(0.686, -0.572, 0.208) # above block # need to start here so joint angles are more consistent 
        # move_to(0.673, -0.570, -0.13) # down low 
        # move_to(0.673, -0.470, -0.14) # move 10 centimeters right  # CHANGE MIDDLE ONE for symmetric position

        # rospy.sleep(1) # so joint angles have more than enough time to get situated
        # if joint_angles is not None: # turn the gripper 
        #         
        # else :
        #      print("no joint angles found bud")

        # read in the joint angles at the desired final position


        # # go to setup position
        os.system(f"rosrun intera_examples go_to_joint_angles.py -q -1.4944794921875 0.5608037109375 -1.655158203125 -1.43123828125 0.968298828125 1.3673369140625 1.766833984375")   


        move_to(0.656, -0.371, 0.00) # up 8 cm in same position
        
        # read xyz so we can use move it

        rospy.sleep(1) 
        if joint_angles is not None: # turn the gripper 
             my_joints = joint_angles
             print(my_joints)
             # make putter be in the right orientation
             
        #      #setup position

             
             
        #      # swing back
        #      os.system(f"rosrun intera_examples go_to_joint_angles.py -q {my_joints[0]} {my_joints[1]} {my_joints[2]} {my_joints[3]} {my_joints[4] } {my_joints[5] - np.pi/16} {my_joints[6]}")   
        #      # swing
        #      os.system(f"rosrun intera_examples go_to_joint_angles.py -q {my_joints[0]} {my_joints[1]} {my_joints[2]} {my_joints[3]} {my_joints[4] } {my_joints[5] + np.pi/16 } {my_joints[6] }")   
        # else :
        #      print("no joint angles found bud")


        
        # set it so just ONE Of the joint angles changes.....  incrementally ... back and then forward? 


        # move_to(0.786, -0.368, 0.211) # above block
        # move_to(0.916, -0.132, 0.245) # above dropoff location
        # move_to(0.862, -0.089, -0.135) # drop off position

        # Open the right gripper
        print('Opening...')
        right_gripper.open()
        rospy.sleep(1.0)
        print('Done!')



# Python's syntax for a main() method
if __name__ == '__main__':
    main()
