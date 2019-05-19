''' This is for live drawing with dodecapen'''

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



def main():
	plot_switch = 1
	hist_plot_switch = 0
	data = dodecapen.txt_data()
	params = dodecapen.parameters()
	ic = dodecapen. get_pnt_grey_image()
	tip_loc_cent = np.array([0.0,0.0,120.0,1]).reshape(4,1)
	
	pose_marker_without_opt = np.zeros((10000,6))
	pose_marker_with_APE = np.zeros((10000,6))
	pose_marker_with_DPR = np.zeros((10000,6))
	tip_position = np.zeros((10000,3))
	tip = np.zeros((10000,6))
	
	j = 0 
	while(True):
 		frame = ic.cv_image
		if frame is None:
			time.sleep(0.1)
			print("No image")

			continue
		

		frame_gray_draw,pose_without_opt, pose_APE,pose_DPR,visib_flag = dodecapen.find_pose(frame,params,data)
		if visib_flag == 1:
			pose_marker_with_APE[j,:] = pose_APE
			pose_marker_without_opt[j,:] = pose_without_opt
			pose_marker_with_DPR[j,:] = pose_DPR

			tf_cam_to_cent = dodecapen.RodriguesToTransf(pose_DPR)

			tip_loc_cam = tf_cam_to_cent.dot(tip_loc_cent)
			# print 	tip_loc_cam[0:3].shape	
			tip_position[j,:] = tip_loc_cam[0:3].reshape(3,) 
 
			tip_pix,_ = cv2.projectPoints(tip_loc_cam[0:3].reshape(1,3),np.zeros((3,1)),np.zeros((3,1)),
											  params.mtx,params.dist)
			
			center = tuple(np.ndarray.astype(tip_pix[0,0],int))
			print center,"center"
			# cv2.circle( frame_gray_draw, center, 20 , 127, -1)

			print("frame number ", j)
			cv2.imshow('frame_gray_draw',frame_gray_draw)
			
			if cv2.waitKey(0) & 0xFF == ord('r') :
				print "recording"
				j+=1

			if cv2.waitKey(0) & 0xFF == ord('q') :	
				print "stopped"
				break

	pose_marker_without_opt = pose_marker_without_opt[0:j,:]
	pose_marker_with_APE = pose_marker_with_APE[0:j,:]
	pose_marker_with_DPR = pose_marker_with_DPR[0:j,:]
	tip_position = tip_position[0:j,:]
	cv2.destroyAllWindows()




	if plot_switch == 1 : 
		r2d = 180/np.pi
		### translation
		fig = plt.figure()

		ax = fig.add_subplot(111, projection="3d")
		fig.canvas.set_window_title("translation x,y,z") 

		ax.set_xlabel('X Label')
		ax.set_ylabel('Y Label')
		ax.set_zlabel('Z Label')
		ax.scatter(pose_marker_without_opt[:,3],pose_marker_without_opt[:,4],pose_marker_without_opt[:,5],
							c ='m',label = "pose_marker_without_opt")
		ax.scatter(pose_marker_with_APE[:,3],pose_marker_with_APE[:,4],pose_marker_with_APE[:,5],
							c = 'r',label="pose_marker_with_APE" )
		ax.scatter(pose_marker_with_DPR[:,3],pose_marker_with_DPR[:,4],pose_marker_with_DPR[:,5],
							c = 'g',label="pose_marker_with_DPR" )
		ax.legend()
		
		### rotation
		# fig = plt.figure()
		# fig.canvas.set_window_title("rotation x,y,z") 

		# ax.set_xlabel('X Label')
		# ax.set_ylabel('Y Label')
		# ax.set_zlabel('Z Label')
		# ax = fig.add_subplot(111, projection="3d")
		# ax.scatter(pose_marker_without_opt[:,0]*r2d, pose_marker_without_opt[:,1]*r2d, pose_marker_without_opt[:,2]*r2d,
		# 					c ='m',label = "orientation_marker_without_opt")
		# ax.scatter(pose_marker_with_APE[:,0]*r2d, pose_marker_with_APE[:,1]*r2d, pose_marker_with_APE[:,2]*r2d,
		# 					c = 'r',label="orientation_marker_with_APE" )
		# ax.scatter(pose_marker_with_DPR[:,0]*r2d, pose_marker_with_DPR[:,1]*r2d, pose_marker_with_DPR[:,2]*r2d,
		# 					c = 'g',label="orientation_marker_with_DPR" )
		# ax.legend()


		### tip 
		fig = plt.figure()
		fig.canvas.set_window_title("tip x,y,z") 

		ax = fig.add_subplot(111, projection="3d")
		ax.set_xlabel('X Label')
		ax.set_ylabel('Y Label')
		ax.set_zlabel('Z Label')
		ax.scatter(tip_position[:,0], tip_position[:,1], tip_position[:,2],
							c ='k',label = "tip_position")
	 

		if hist_plot_switch == 1:
			## translation 	
			fig = plt.figure()
			fig.canvas.set_window_title("histogram translation z") 
			plt.hist(pose_marker_without_opt[:,5],j,facecolor='magenta',normed = 1,label = 'pose_marker_without_opt' )
			plt.hist(pose_marker_with_APE[:,5],j,facecolor='red',normed = 1, label = 'pose_marker_with_APE'  )
			plt.hist(pose_marker_with_DPR[:,5],j,facecolor='green',normed = 1, label = 'pose_marker_with_DPR'  )
			plt.legend()
			
			## rotation
			# fig = plt.figure()
			# fig.canvas.set_window_title("histogram rotation z") 
			# plt.hist(pose_marker_without_opt[:,2]*r2d,j,facecolor='magenta',normed = 1,label = 'orientation_marker_without_opt' )
			# plt.hist(pose_marker_with_APE[:,2]*r2d,j,facecolor='red',normed = 1, label = 'orientation_marker_with_APE'  )
			# plt.hist(pose_marker_with_DPR[:,2]*r2d,j,facecolor='green',normed = 1, label = 'orientation_marker_with_DPR'  )
			# plt.legend()

			print ("the end")

	plt.show()


if __name__ == '__main__':
	
	main()