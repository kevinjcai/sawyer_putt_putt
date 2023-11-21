# Current Commands

- rosrun putt_putt ball_detection.py: RUN IN IMAGE DIRECTORY! records pictures and finds golf ball, saves the annotated image with the found circles into your current working directory

# Launch Files
- roslaunch putt_putt camera_tuck.launch: brings you to a HIGH UP scanning position so you can see a horizontal table
- roslaunch putt_putt swing_start_tuck.launch: brings you to a potential swinging-like position like a foot from the table

# AR Tag Sensing
- rosrun intera_interface joint_trajectory_action_server.py  # to set up the intera action server
- roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true # to start MoveIt through rviz
- roslaunch putt_putt sawyer_camera_track.launch # to start AR tracking node
- rosrun putt_putt main.py -ar_marker [NUM] # works consistently with tuck() from lab7, gives weird nulltype errors from camera_tuck()
