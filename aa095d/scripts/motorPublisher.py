#!/usr/bin/env python3

import rospy
from lib import detectLine
from std_msgs.msg import UInt16MultiArray

def publish():
    pub = rospy.Publisher('Motor', UInt16MultiArray, queue_size = 10)
    rospy.init_node('motorPublisher', anonymous = True)
    rospy.loginfo('Start motorPublisher')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        motorMsg = UInt16MultiArray()
        # 90: straight
        # 40: turn left max
        # 135: turn right max
        motorServo = 90
        # ~ 255 max
        motorSpeed = 125
        # 2: forward
        # 1: halt
        # 0: backward
        motorWay = 2

        motorMsg.data = [motorServo, motorSpeed, motorWay]
        pub.publish(motorMsg)
        rate.sleep()

if __name__ == '__main__':
    try:
        #detectLine.runDetector("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, format=(string)NV12, framerate=(fraction)60/1 ! nvvidconv flip-method=0 ! video/x-raw, width=(int)640, height=(int)360, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink")
        detectLine.runDetector('videotestsrc ! video/x-raw,framerate=20/1 ! videoscale ! videoconvert ! appsink')
        #publish()
    except rospy.ROSInterruptException:
        pass

