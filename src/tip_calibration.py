import DoDecahedronUtils  as dodecapen
import numpy as np
from numpy import linalg as LA
import cv2
import cv2.aruco as aruco
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation 
from mpl_toolkits.mplot3d import Axes3D
import transforms3d as tf3d
import time
from scipy.interpolate import griddata

#from helper import *
from scipy.optimize import minimize, leastsq,least_squares
from scipy import linalg
from scipy.spatial import distance
import rospy
from roscam import RosCam
from matplotlib.path import Path

def std_tip_pos(tip_pos_gs,pose_marker_with_DPR):
	tip_pos_gs = np.append(tip_pos_gs,1.0)
	n = pose_marker_with_DPR.shape[0]
	tip_position = np.zeros((n,3))
	for j in range(n):

		tf_cam_to_cent = dodecapen.RodriguesToTransf(pose_marker_with_DPR[j,:])
		tip_loc_cam = tf_cam_to_cent.dot(tip_pos_gs.reshape(4,1))
		tip_position[j,:] = tip_loc_cam[0:3].reshape(3,) 

	a = tip_position - tip_position.mean(axis = 0)
	b = np.linalg.norm(a,axis=1)	
	# print tip_position,"tip_position"
	print tip_pos_gs,"tip_pos_gs"
	print b.sum(),"b.sum()"
	return b.sum()
	

def main():
	plot_switch = 1
	hist_plot_switch = 0
	data = dodecapen.txt_data()
	params = dodecapen.parameters()
	tip_loc_cent = np.array([-1.19893687e-01,  1.56143014e+00 , 1.21730196e+02]).reshape(3,)
	pose_marker_without_opt = np.zeros((10000,6))
	pose_marker_with_APE = np.zeros((10000,6))
	pose_marker_with_DPR = np.zeros((10000,6))
	tip_position = np.zeros((10000,3))
	tip = np.zeros((10000,6))

	cap = cv2.VideoCapture('pen_tip_calib.avi')
	ret = 1
	j = 0 
	while(ret):
		
 		ret,frame = cap.read()
		if frame is None:
			time.sleep(0.1)
			print("No image")

			continue
		

		frame_gray_draw,pose_without_opt, pose_APE,pose_DPR,visib_flag = dodecapen.find_pose(frame,params,data)
		if visib_flag == 1:
			pose_marker_with_APE[j,:] = pose_APE
			pose_marker_without_opt[j,:] = pose_without_opt
			pose_marker_with_DPR[j,:] = pose_DPR

 
			# tip_pix,_ = cv2.projectPoints(tip_loc_cam[0:3].reshape(1,3),np.zeros((3,1)),np.zeros((3,1)),
			# 								  params.mtx,params.dist)
			
			# center = tuple(np.ndarray.astype(tip_pix[0,0],int))
 
			# cv2.circle( frame_gray_draw, center, 20 , 127, -1)

			print("frame number ", j)
			cv2.imshow('frame_gray_draw',frame_gray_draw)

			if cv2.waitKey(1) & 0xFF == ord('q') & ret:	
				print "stopped"
				break
			j+=1	

	pose_marker_without_opt = pose_marker_without_opt[0:j,:]
	pose_marker_with_APE = pose_marker_with_APE[0:j,:]
	pose_marker_with_DPR = pose_marker_with_DPR[0:j,:]
	tip_position = tip_position[0:j,:]
	cv2.destroyAllWindows()
	print std_tip_pos(tip_loc_cent,pose_marker_with_DPR)
	print std_tip_pos(np.zeros((3,)),pose_marker_with_DPR)

	print "optimizing "

	res = minimize(std_tip_pos,tip_loc_cent,args = pose_marker_with_DPR)

	print res.x, "calibration params"

if __name__ == '__main__':
	
	main()
