# Current Commands

- rosrun putt_putt ball_detection.py: RUN IN IMAGE DIRECTORY! records pictures and finds golf ball, saves the annotated image with the found circles into your current working directory

# Launch Files
- roslaunch putt_putt camera_tuck.launch: brings you to a HIGH UP scanning position so you can see a horizontal table
- roslaunch putt_putt swing_start_tuck.launch: brings you to a potential swinging-like position like a foot from the table

# AR Tag Sensing
- rosrun intera_interface joint_trajectory_action_server.py  
    - to set up the intera action server
- roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true
    - to start MoveIt through rviz
- roslaunch putt_putt sawyer_camera_track.launch 
    - to start AR tracking node
- rosrun putt_putt main.py -ar_marker [NUM] 
    - works consistently with tuck() from lab7, gives weird nulltype errors from camera_tuck()


# MOVE IT Lab 5 Stuff
 - rosrun intera_interface joint_trajectory_action_server.py
 - roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true
 - START WITH roslaunch intera_examples sawyer_tuck.launch to get position going 
 - always attatch putter with the flat part of the putter facing the same way as the camera, use holes 1 and 3 (further from camera)


 # getting tip position
 rosrun tf tf_echo base right_gripper_tip



 # running camera
 - rosrun intera_examples camera_display.py -c right_hand_camera

 
 # stuff to know
 - 515 pixels =  1 meter basically
 



# POSITION THINGS
- robot: 
    * x is away from the robot towards computers
    * y is away from the TA desk towards door


- camera:
    * x is towards the robot away from computers
    * y is away from the TA desk towards door