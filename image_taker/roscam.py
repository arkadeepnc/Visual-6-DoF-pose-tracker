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
 

class RosCam:
  def __init__(self, topic):
    self.bridge = CvBridge()
    self.cv_image = None
    self.image_sub = rospy.Subscriber(topic,Image,self.callback)
    

  def callback(self,data):
    try:
      self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

def main(args):
  import time

  print("hello")
  rospy.init_node('RosCam', anonymous=True)
  ic = RosCam("/camera/image_color")

  while True:
    if ic.cv_image is None:
      time.sleep(0.1)
      continue
    try:
      print(ic.cv_image[0,0])
      cv2.imshow("Image window", ic.cv_image)
      if cv2.waitKey(1) & 0xFF == ord('q'):
        break
      
    except KeyboardInterrupt:
      print("Shutting down")
      break
  cv2.destroyAllWindows()

# def listener():
#   main(sys.argv)
#   rospy.init_node('listener',anonymous=True)
#   Image = rospy.Subscriber('image_topic_2',Image)
#   return Image

if __name__ == '__main__':
  main(sys.argv)
 