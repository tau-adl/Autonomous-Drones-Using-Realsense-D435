#!/usr/bin/env python

import rospy
import crazyflie
from geometry_msgs.msg import PoseStamped


def onNewTransform(pose):
	global cfR
	global counter
	global ref_rate
	global ref_samples
	green_pose = [pose.pose.position.x , pose.pose.position.y , pose.pose.position.z]
	print(green_pose)
	counter+= 1
	if(counter == ref_samples):
		cfR.land(targetHeight = 0.0, duration = 2.0)
		rospy.sleep(3.0)
		cfR.stop()
	elif(counter < ref_samples):
		cfR.goTo(goal = green_pose, yaw=0.0, duration = ref_rate, relative = False)
	rospy.sleep(ref_rate)

if __name__ == '__main__':
	rospy.init_node('test_high_level')

	cfR = crazyflie.Crazyflie("crazyflieR", "crazyflieR")
	cfR.setParam("commander/enHighLevel", 1)

	counter = 0
	ref_rate = 1.0
	ref_samples = 30
	cfR.takeoff(targetHeight = 0.1 ,duration = 2.0)
	rospy.sleep(3.0)
	rospy.Subscriber("/ball_poseG", PoseStamped, onNewTransform , queue_size = 1)
	rospy.spin()