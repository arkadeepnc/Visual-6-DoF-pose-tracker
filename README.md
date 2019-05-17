# Visual-6-DoF-pose-tracker
This is a repository on top of the DodecaPen base code.
This code is in Python 2.7 but we would like to have the option to port it to Python 3.5 or higher. 
For now please write python3 compatible code by adding the folowing lines on top:
from __future__ import division
from __future__ import print_function
However, do not run the code in python3 as you may have some problems woth ROS OpenCV distro.

# REQUIREMENTS
UBUNTU 16.04 LTS [ not sure if it works with virtual Linux machine in Windows ]
ROS KINETIC
ros point_grey_camera_driver 
Read the Docs here: http://wiki.ros.org/pointgrey_camera_driver

Install prebuilt bins from source :

sudo apt-get install ros-kinetic-pointgrey-camera-driver

Connect camera to USB 3.1 (blue port). USB 2 would not work!! Start camera with : 

$ roslaunch pointgrey_camera_driver camera.launch

Check feed 
$ rostopic echo /camera/image_color 

should show a bunch of numbers in the screen within seconds after hitting return! Or else rostopic is dead. 

Library dependencies:  [all are basic and should be there if your Ubuntu-ROS do not hate each other]

numpy (latest)
OpenCV contrib (test it as with the following in the terminal: >python -> >import  cv2 -> >from cv2 import aruco)
rospy
matplotlib
transforms3d (pip install transforms3d)
scipy (check pypi for latest distro)
snakeviz -- for profiling your code






