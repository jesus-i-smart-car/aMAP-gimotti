#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2

def callback(msg):
    rospy.loginfo('asdf')

def subscribe():
    rospy.init_node('imageSubscriber')
    rospy.Subscriber('camera/image_raw', Image, callback, queue_size = 1, buff_size = 16777216)
    rospy.spin()

if __name__ == '__main__':
    subscribe()

