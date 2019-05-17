''' Use this code to check camera feed'''


from __future__ import division
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


sub_pix_refinement_switch = 3
detect_tip_switch = 0
hist_plot_switch = 1
show_filter_switch = 1
show_ape_switch = 0

iterations_for_while =5500
marker_size_in_mm = 17.78
distance_betn_markers = 34.026  #in mm
dilate_fac = 1.2 # dilate the square around the marker
tip_coord  = np.array([3.97135363, -116.99921549 ,-5.32922903,1]) 

pose_marker_with_APE= np.zeros((iterations_for_while,6))
pose_marker_with_DPR= np.zeros((iterations_for_while,6))
pose_marker_without_opt = np.zeros((iterations_for_while,6))
#pose_marker_avg = np.zeros((iterations_for_while,6))

tip_posit = np.zeros((iterations_for_while,3))
color = [0]*iterations_for_while


aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
parameters =  aruco.DetectorParameters_create()
parameters.cornerRefinementMethod = sub_pix_refinement_switch
parameters.cornerRefinementMinAccuracy = 0.05
stkd_2d_corn_pix_sp = np.zeros((12,2))
markers_possible = np.array([1,2,3,4,5,6,7,8,9,10,11,12])
markers_impossible = np.array([13,17,37,16,34,45,38,24,47,32,40])
points_for_DPR = 10

j = 0  # iteration counter


time_vect = [0]*iterations_for_while


with np.load('PTGREY.npz') as X:
	mtx, dist = [X[i] for i in ('mtx','dist')] 
 


rospy.init_node('RosCam', anonymous=True)
ic = RosCam("/camera/image_color")

t_prev = time.time()   	

while(True):
	t_new = time.time()   

	frame = ic.cv_image
	# print(frame)
	if frame is None:
		time.sleep(0.1)
		print("No image")
		continue
	else:
		frame_gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
		 
		print("frame number ", j)
		cv2.imshow('frame_color',frame)
		# t1 = time.time() - t0
		print("current frabme rate",1./(t_new- t_prev))
		j = j+1
		if cv2.waitKey(1) & 0xFF == ord('q') :
			break


	t_prev = t_new
	 


cv2.destroyAllWindows()



#### Analysis

