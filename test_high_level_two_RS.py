#!/usr/bin/env python

import rospy
import crazyflie
import uav_trajectory

def test_one_takeoff(cf, param = 0.3):
	cf.takeoff(targetHeight = param ,duration = 3.0)
	rospy.sleep(10.0)
	cf.land(targetHeight = 0.0, duration = 3.0)

def test_one_basic(cf, param = 0.3):

	cf.takeoff(targetHeight = param ,duration = 2.0)
	rospy.sleep(6.0)

	cf.goTo(goal = [0.0, param, 0.0], yaw=0.0, duration = 2.0, relative = True)
	rospy.sleep(5.0)

	cf.goTo(goal = [0.0, -2*param, 0.0], yaw=0.0, duration = 4.0, relative = True)
	rospy.sleep(7.0)

	cf.goTo(goal = [0.0, param, 0.0], yaw=0.0, duration = 2.0, relative = True)
	rospy.sleep(5.0)

	cf.land(targetHeight = 0.0, duration = 2.0)

def test_one_rect(cf, param = 0.3):

	cf.takeoff(targetHeight = param ,duration = 2.0)
	rospy.sleep(2.0)

	#cf.goTo(goal = [-param/2, 0.0, 0.0], yaw=0.0, duration = 2.0, relative = True)
	#rospy.sleep(2.0)

	for i in range(2):
		cf.goTo(goal = [-2*param, param, 0.0], yaw=0.0, duration = 4.0, relative = True)
		rospy.sleep(4.0)

		cf.goTo(goal = [0.0, -2*param, 0.0], yaw=0.0, duration = 4.0, relative = True)
		rospy.sleep(6.0)

		cf.goTo(goal = [2*param, param, 0.0], yaw=0.0, duration = 4.0, relative = True)
		rospy.sleep(4.0)

	#cf.goTo(goal = [param/2, 0.0, 0.0], yaw=0.0, duration = 2.0, relative = True)
	#rospy.sleep(2.0)

	cf.land(targetHeight = 0.05, duration = 3.0)
	

def test_both_takeoff(left , right , height = 0.3):
	left.takeoff(targetHeight = height ,duration = 3.0)
	right.takeoff(targetHeight = height ,duration = 3.0)
	rospy.sleep(10.0)

	left.land(targetHeight = 0.0, duration = 3.0)
	right.land(targetHeight = 0.0, duration = 3.0)


def test_both_square(left , right):
	height = 0.3
	length = 0.5
	diff = 0.1
	dur = 10 * length
	repeats = 2
	left.takeoff(targetHeight = height + diff,duration = 10 * (height + diff))
	right.takeoff(targetHeight = height ,duration = 10 * height)
	rospy.sleep(10 * (height + diff) + 1)

	right.goTo(goal = [ -length , 0.0 , 0.0], yaw=0.0, duration = dur, relative = True)
	rospy.sleep(dur)

	for i in range(repeats):

		left.goTo(goal = [ 0.0 , length , 0.0], yaw=0.0, duration = dur, relative = True)
		right.goTo(goal = [ 0.0 , -length , 0.0], yaw=0.0, duration = dur, relative = True)
		rospy.sleep(dur)	

		left.goTo(goal = [-length , 0.0 , 0.0], yaw=0.0, duration = dur, relative = True)
		right.goTo(goal = [length , 0.0 , 0.0], yaw=0.0, duration = dur, relative = True)
		rospy.sleep(dur)

		left.goTo(goal = [0.0 , 0.0 , -diff], yaw=0.0, duration = 20 * diff, relative = True)
		right.goTo(goal = [0.0 , 0.0 , diff], yaw=0.0, duration = 20 * diff, relative = True)
		rospy.sleep(20 * diff)


		left.goTo(goal = [ 0.0 , -length , 0.0], yaw=0.0, duration = dur, relative = True)
		right.goTo(goal = [ 0.0 , length , 0.0], yaw=0.0, duration = dur, relative = True)
		rospy.sleep(dur)

		left.goTo(goal = [length , 0.0 , 0.0], yaw=0.0, duration = dur, relative = True)
		right.goTo(goal = [-length , 0.0 , 0.0], yaw=0.0, duration = dur, relative = True)
		rospy.sleep(dur)

		left.goTo(goal = [0.0 , 0.0 , diff], yaw=0.0, duration = 20 * diff, relative = True)
		right.goTo(goal = [0.0 , 0.0 , -diff], yaw=0.0, duration = 20 * diff, relative = True)
		rospy.sleep(20 * diff)

	right.goTo(goal = [ length , 0.0 , 0.0], yaw=0.0, duration = dur, relative = True)
	rospy.sleep(dur)

	left.land(targetHeight = 0.1, duration = 10 * (height + diff))
	right.land(targetHeight = 0.1, duration = 10 * height)
	rospy.sleep(10 * (height + diff))



if __name__ == '__main__':
	rospy.init_node('test_two_high_level')

	cfR = crazyflie.Crazyflie("crazyflieR", "crazyflieR")
	cfR.setParam("commander/enHighLevel", 1)

	#cfG = crazyflie.Crazyflie("crazyflieG", "crazyflieG")
	#cfG.setParam("commander/enHighLevel", 1)

	#test_one_basic(cfG)
	#test_one_basic(cfR)

	#test_one_rect(cfG)
	test_one_rect(cfR)

	#test_one_takeoff(cfG)
	#test_one_takeoff(cfR)

	#test_both_takeoff(cfG,cfR)
	#test_both_square(cfG,cfR)

	rospy.sleep(5.0)
	cfR.stop()
	#cfG.stop()