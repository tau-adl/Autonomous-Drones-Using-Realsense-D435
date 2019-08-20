# Autonomous-Drones-Using-Realsense-D435

The project runs over ubuntu 16.04.

# Prerequests:
1. ROS Kinetic - (http://wiki.ros.org/kinetic/Installation/Ubuntu)
2. Realsense SDK 2.0 - (https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)
3. Crazyflie (ROS Wrapper, Python Wrapper) - (http://act.usc.edu/publications/Hoenig_Springer_ROS2017.pdf) , (https://github.com/whoenig/crazyflie_ros)
4. Python packages: rospy (installed with ROS-Kinetic), OpenCV2.0 (https://pypi.org/project/opencv-python/), pyrealsense2 (https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python).

# Running The Project:
* After installing the prerequests, add the project's code files to "/crazyflie_ws/src/crazyflie_demo/scripts/".
* To run the system using 2 drones, run "start_crazyflie_RS_two.sh", and then run "test_high_level_two_RS.py"
* To run the system using 1 drone, run "start_crazyflie_RS_stick.sh", and then run "test_high_level_stick_RS.py"

# Note: There a sample video of the project in /Project_Docs.
