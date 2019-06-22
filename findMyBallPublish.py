#!/usr/bin/env python

# import the necessary packages
from collections import deque
import numpy as np
import cv2
import imutils
import time
import pyrealsense2 as rs
import rospy
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt


# buffers' sizes
buff_size = 32
axes_buff_size = 512

# colored balls' indexes
GNum = 0
RNum = 1

# HSV ranges of balls' colors
lowerG = (29, 86, 6)
upperG = (64, 255, 255)
lowerR = (0,50,50)
upperR = (10,255,255)
lowerR2 = (170,50,50)
upperR2 = (180,255,255)

# iterations counter - used for initialization and debugging
iterNum = 0

# initilized axes
axes_o = [(0,0,0),(0,0,0)]

# axes deques used for balls tracking, initialization, factoring and debugging
axes = [deque(maxlen = axes_buff_size),deque(maxlen = axes_buff_size)]

# pixel axes deque for tracked line drawing
pts = [deque(maxlen = buff_size),deque(maxlen = buff_size)]

# allow the camera to warm up
time.sleep(2.0)

# create, config and start realsense streaming pipeline
pipeline = rs.pipeline()
config = rs.config()
# RGB and depth:
# Resolution 640 x 480 (for faster proccessing, default of depth is 1080p)
# FPS : 30
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
pipeline.start(config)

# ROS node initialization
rospy.init_node('ball_poses_publish', anonymous=True)

# ROS positions' publishers
pub = [rospy.Publisher("ball_poseG", PoseStamped, queue_size=1) ,
		rospy.Publisher("ball_poseR", PoseStamped, queue_size=1)]

''' Input: (x,y,z) - colored ball's axes
		   ballNum - colored ball's index (Green = 0 , Red = 1) 
	The function gets the colored ball's axes and index, and publishes the axes
	to the colored ball's topic (based on the index) in PoseStamped format.
'''
def publish_pose_stamped(x,y,z,ballNum):
	rospy.loginfo("CF [{}] Axes : X : [{}] , Y : [{}] , Z : [{}]".format(ballNum,x,y,z))
	pose = PoseStamped()
	pose.header.stamp = rospy.Time.now()
	pose.header.frame_id = "ball_pose_{}".format(ballNum)
	pose.pose.position.x = (x)
	pose.pose.position.y = (y)
	pose.pose.position.z = (z)
	pub[ballNum].publish(pose)

''' Input: (x,y,z) - colored ball's axes
			ballNum - colored ball's index
	This function handles the axes initialization and storing process.
	Output: a boolean value that mentions whether the axes are now usable or not.
'''
def update_publish_axes(x , y , z , ballNum):

	if 50 < iterNum < 100:
		axes[ballNum].append( (x , y , z) )

	if iterNum == 100:
		axes_o[ballNum] = np.mean(list(axes[ballNum]) , axis = 0)
		axes[ballNum].clear()

	if iterNum > 100:
		dx , dy , dz = (x - axes_o[ballNum][0] , y - axes_o[ballNum][1] , z - axes_o[ballNum][2])
		axes[ballNum].append( (dx, dy, dz) )
		# Crazyflie Params: x is -z, y is x, z is -y (Crazyflie faces to camera)
		publish_pose_stamped(-dz, dx, -dy, ballNum)

''' Input: frame - the original colored frame
			(x,y,radius) - pixel axes and radius of the found colored ball
			ballNum - colored ball's index
	This function marks the found ball over the frame with green circle and draws
	a red line over the movement path of this colored ball.
'''
def markBallInFrame(frame , x , y , radius , ballNum):
	# draw the circle, centroid and line on the frame
	cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
	pts[ballNum].appendleft((int(x), int(y)))
	# loop over the set of tracked points to draw the line
	for i in range(1, len(pts[ballNum])):
		if pts[ballNum][i - 1] is None or pts[ballNum][i] is None:
			continue
		cv2.line(frame, pts[ballNum][i - 1], pts[ballNum][i], (0, 0, 255)
			, int(np.sqrt(buff_size / float(i + 1)) * 2.5))
	cv2.circle(frame, (int(x), int(y)), 2, (255, 0, 0), 7)

''' Input :	cnts - ball's contours
			depth - depth frame
			ballNum - colored ball's index
			graph_iter - Number of iterations before outputting
						 axes graph (used for debugging)
			factor - averaging factor
	This function handles the main processing sequence: contour's axes gathering
	and initialization, frame's marking, average factoring, axes reordering and publishing,
	and debugging features handling. 
'''
def ProccessCircle(frame , cnts , depth , ballNum , graph_iter = 0):

	# only proceed if at least one contour was found
	if len(cnts) > 0:

		((x, y), radius) = cv2.minEnclosingCircle(max(cnts, key=cv2.contourArea))

		# only proceed if the radius meets a minimum size
		if radius > 5:

			# draw the circle, centroid and line on the frame
			markBallInFrame(frame , x ,y , radius , ballNum)

			# convert from pixels coordinates to real world cordinates
			depth_val = depth.get_distance(int(x),int(y))
			depth_intrin = depth.profile.as_video_stream_profile().intrinsics
			(x , y , z) = rs.rs2_deproject_pixel_to_point(depth_intrin , [x,y] , depth_val)

			# update x , y , z according to initial state and publish
			update_publish_axes(x , y , z , ballNum)

			# build and output graphes if needed (debugging purposes)
			if graph_iter > 0:
				if iterNum == graph_iter:
					iters = [i for i in range(len(axes[ballNum]))]
					plt_axes = zip(*axes[ballNum])
					plt.plot(iters , list(plt_axes[0]), 'r--', iters , list(plt_axes[1]), 'bs', iters , list(plt_axes[2]), 'g^')
					plt.show()

''' The Function draws and aligns the RGB and depth frames from the Realsense
	pipeline, and returns the aligned RGB and depth frames 
'''
def get_aligned_rgb_depth():
	# grab the current frame -> rgb and depth
	curr_frame = pipeline.wait_for_frames()

	# Create alignment primitive with **depth** as its target stream:
	align = rs.align(rs.stream.depth)
	curr_frame = align.process(curr_frame)

	depth = curr_frame.get_depth_frame().as_depth_frame()
	rgb = curr_frame.get_color_frame()
	if not (rgb and depth):
		return (None , None)

	rgb = np.asanyarray(rgb.get_data())
	return (rgb , depth)

''' Input: hsv - An HSV frame
		   HSV range (or ranges) of the wanted color
	The Function creates a mask of the HSV frame over the given bounds, and
	returns all the Contours in the mask
'''
def get_contour_from_hsv(hsv , lowerBound , upperBound , lowerBound2 = None , upperBound2 = None , show_mask = False):
	# construct a mask for the color "green", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
	mask = cv2.inRange(hsv, lowerBound, upperBound)

	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)

	if not (lowerBound2 == None or upperBound2 == None):
		mask2 = cv2.inRange(hsv, lowerBound2, upperBound2)
		mask2 = cv2.erode(mask2, None, iterations=2)
		mask2 = cv2.dilate(mask2, None, iterations=2)
		mask = cv2.bitwise_or(mask, mask2)
	if show_mask is True:	
		cv2.imshow("Mask", mask)

	# find contours in the mask
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	
	return cnts

time.sleep(3.0)

# keep looping
while True:
	iterNum += 1
	
	rgb , depth = get_aligned_rgb_depth()
	if (depth is None):
		continue
		
	# set BGR frame standard, resize, blur, and convert it to HSV
	frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
	blurred = cv2.GaussianBlur(frame, (11, 11), 0)

	# get hsv frames
	hsvG = cv2.cvtColor(blurred, cv2.COLOR_RGB2HSV)
	hsvR = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

	cntsG = get_contour_from_hsv(hsvG , lowerG , upperG , show_mask = False)
	cntsR = get_contour_from_hsv(hsvR, lowerR , upperR, lowerBound2 = lowerR2 , upperBound2 = upperR2 , show_mask = False)

	ProccessCircle(frame , cntsG , depth , GNum , graph_iter = 0)
	ProccessCircle(frame , cntsR , depth , RNum , graph_iter = 0)
		
	# show the frame to our screen
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF
 
	# if the 'q' key is pressed, stop the loop
	if key == ord("q"):
		break

# close all windows
cv2.destroyAllWindows()
