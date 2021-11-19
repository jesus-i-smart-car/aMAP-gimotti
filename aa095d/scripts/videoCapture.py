#!/usr/bin/env python

import rospy
import cv2, time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from threading import Thread

def gstreamer_pipeline():
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, format=(string)NV12, framerate=(fraction)60/1 ! nvvidconv flip-method=2 ! video/x-raw, width=(int)640, height=(int)360, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink"

class ThreadedCamera(object):
	def __init__(self, src=0):
		self.cap = cv2.VideoCapture(src)
		self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
		self.FPS = 1/30
		self.FPS_MS = int(self.FPS * 1000)
		self.thread = Thread(target=self.update, args=())
		self.thread.daemon= True
		self.thread.start()
	def update(self):
		while True:
			if self.cap.isOpened():
				(self.status, self.frame) = self.cap.read()
			time.sleep(self.FPS)
	def showFrame(self):
		cv2.imshow('frame', self.frame)
		cv2.waitKey(self.FPS_MS)

if __name__ == '__main__':
	threadedCamera = ThreadedCamera(gstreamer_pipeline())
	rospy.init_node("videoCapture", anonymous=True)
	while True:
		try:
			threadedCamera.showFrame()
		except AttributeError:
			pass

