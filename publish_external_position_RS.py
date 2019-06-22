#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PointStamped, TransformStamped, PoseStamped #PoseStamped added to support vrpn_client
from crazyflie_driver.srv import UpdateParams

def onNewTransform(pose):
    global msg
    global pub
    global firstTransform

    if firstTransform:
        # initialize kalman filter
        rospy.set_param("kalman/initialX", pose.pose.position.x) # Change pose according to your topic
        rospy.set_param("kalman/initialY", pose.pose.position.y)
        rospy.set_param("kalman/initialZ", pose.pose.position.z)
        update_params(["kalman/initialX", "kalman/initialY", "kalman/initialZ"])

        rospy.set_param("kalman/resetEstimation", 1)
        update_params(["kalman/resetEstimation"]) 
        firstTransform = False

    else:
        msg.header.frame_id = pose.header.frame_id # Change pose according to your topic
        msg.header.stamp = pose.header.stamp
        msg.header.seq += 1
        msg.point.x = pose.pose.position.x 
        msg.point.y = pose.pose.position.y
        msg.point.z = pose.pose.position.z
        pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('publish_external_position_RS', anonymous=True)
    topic = rospy.get_param("~topic")
    rospy.wait_for_service('update_params')
    rospy.loginfo("found update_params service " + topic)
    update_params = rospy.ServiceProxy('update_params', UpdateParams)
    firstTransform = True

    msg = PointStamped()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    pub = rospy.Publisher("external_position", PointStamped, queue_size=1)
    rospy.Subscriber(topic, PoseStamped, onNewTransform)
    rospy.spin()
