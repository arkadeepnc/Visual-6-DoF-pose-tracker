# #!/usr/bin/env python
''' This is a header for ATI force sensor which talks over ROS with wrench stamped rostopics'''
import rospy
import sys

from geometry_msgs.msg import WrenchStamped
import time 


class ati_force:

	def __init__(self, topic):
	# self.bridge = CvBridge()
	# self.cv_image = None
		self.force_data_temp=[]
		self.image_sub = rospy.Subscriber(topic,WrenchStamped,self.callback)
	
	def callback(self,data):
		#print "tests "
  		self.force_data_temp = data.wrench
  		#print self.force_data_temp
  	
  	def get_wrench(self):
  		return self.force_data_temp


def main(args):
	import time

	print("hello")
	ic = ati_force("/atinetft/wrench")
	rospy.init_node('ati_force', anonymous=True)
	force_data_output = ic.get_wrench()
	print(force_data_output)

 
if __name__ == '__main__':
	main(sys.argv)
