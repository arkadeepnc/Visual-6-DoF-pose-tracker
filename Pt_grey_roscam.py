#!/usr/bin/env python
from __future__ import print_function

import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time

# class image_converter:
	# def __init__(self):
	# 	# self.image_pub = rospy.Publisher("image_topic_2",Image)
	# 	self.image_sub = rospy.Subscriber("/camera/image_color",Image,self.callback)

def get_frame():
	frame = rospy.wait_for_message("/camera/image_color", Image, 0.5)
	return frame

def run():
	# ic = image_converter()
	rospy.init_node('image_converter', anonymous=True)
	bridge = CvBridge()
	frame = get_frame()
	img = bridge.imgmsg_to_cv2(frame, "bgr8")
	return img

if __name__ == '__main__':
	cv2.namedWindow('image',cv2.WINDOW_NORMAL)
	while True:
		img = run()
		cv2.imshow("image",img)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
	# cv2.destroyAllWindows()