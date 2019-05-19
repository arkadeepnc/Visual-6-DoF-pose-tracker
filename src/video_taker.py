#Used this code to confirm that the tvec and rvec given by the 
 # estimatePoseSingleMarkers is of the marker frame wrt the camera frame


# from __future__ import division
import numpy as np
from numpy import linalg as LA
import cv2
import cv2.aruco as aruco
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation 
from mpl_toolkits.mplot3d import Axes3D
import transforms3d as tf3d
import time
#from helper import *
from scipy.optimize import minimize, leastsq,least_squares
from scipy import linalg
from scipy.spatial import distance
import rospy
from roscam import RosCam
from geometry_msgs.msg import WrenchStamped
from ati_force import ati_force


 
j = 0  # iteration counter


rospy.init_node('RosCam', anonymous=True)


ic = RosCam("/camera/image_color")

t_prev = time.time()   	
fourcc = cv2.VideoWriter_fourcc(*'XVID')

# wrench_data = np.zeros((100000,6))
# raw_wrench_data = np.zeros((100000,6))

task = 'gamepad_mockup'
# task = 'plane'

# video_file_name = 'vid_recording/'+ task
video_file_name = 'vid_recording/'+ task
out = cv2.VideoWriter(video_file_name +'.avi',fourcc, 30.0, (1280,1024))
# a = ati_force("/atinetft/wrench")
# b = ati_force("/atinetft/raw_wrench")
while(True):

	t_new = time.time()   
	frame = ic.cv_image
 	
	if frame is None:
		# time.sleep(0.1)
		print("No image")
		continue
	else:
		frame_gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)		
		out.write(frame) 
		print("frame number ", j)
		cv2.imshow('frame_color',frame)
		print("current frabme rate",1./(t_new- t_prev))

		# wrench = a.get_wrench()
		# raw_wrench = b.get_wrench()

		# wrench_data[j,0],wrench_data[j,1],wrench_data[j,2] = wrench.force.x,  wrench.force.y,  wrench.force.z
		# wrench_data[j,3],wrench_data[j,4],wrench_data[j,5] = wrench.torque.x,  wrench.torque.y,  wrench.torque.z

		# raw_wrench_data[j,0],raw_wrench_data[j,1],raw_wrench_data[j,2] = raw_wrench.force.x,  raw_wrench.force.y,  raw_wrench.force.z
		# raw_wrench_data[j,3],raw_wrench_data[j,4],raw_wrench_data[j,5] = raw_wrench.torque.x,  raw_wrench.torque.y,  raw_wrench.torque.z		

		j+=1
		if cv2.waitKey(0) & 0xFF == ord('q') :
			print "stopped"
			break
	t_prev = t_new
	 
# wrench_data = wrench_data[0:j,:]
# raw_wrench_data = raw_wrench_data[0:j,:]

# print "."
# print wrench_data,"wrench_data"
# print "."
# print raw_wrench_data,"raw_wrench_data"
# print "."

# np.savetxt(video_file_name+"_wrench"+".txt",wrench_data)
# np.savetxt(video_file_name+"_raw_wrench"+".txt",raw_wrench_data)

out.release()
cv2.destroyAllWindows()
