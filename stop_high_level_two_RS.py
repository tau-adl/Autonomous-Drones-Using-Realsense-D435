#!/usr/bin/env python

import rospy
import crazyflie


if __name__ == '__main__':
	rospy.init_node('test_two_high_level')

	cfR = crazyflie.Crazyflie("crazyflieR", "crazyflieR")
	cfR.setParam("commander/enHighLevel", 1)

	cfG = crazyflie.Crazyflie("crazyflieG", "crazyflieG")
	cfG.setParam("commander/enHighLevel", 1)

	cfR.stop()
	cfG.stop()