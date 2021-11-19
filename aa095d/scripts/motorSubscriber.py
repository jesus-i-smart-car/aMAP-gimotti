#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt16MultiArray

def callback(motor):
    rospy.loginfo("Listening: servo = %d", motor.data[0])
    rospy.loginfo("Listening: speed = %d", motor.data[1])
    rospy.loginfo("Listening: way = %d", motor.data[2])

def subscribe():
    rospy.init_node('motorSubscriber', anonymous = True)
    rospy.Subscriber('Motor', UInt16MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    subscribe()

