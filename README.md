# Visual-6-DoF-pose-tracker
This is a repository on top of the DodecaPen base code.
This code is in Python 2.7 but we would like to have the option to port it to Python 3.5 or higher. 
For now please write python3 compatible code by adding the folowing lines on top:

```from __future__ import division```

```from __future__ import print_function```

However, do not run the code in python3 as you may have some problems woth ROS OpenCV distro.

DodecaPen is based of Po-Chen Wu's thesis work. This is an implematation of the paper __DodecaPen: Accurate 6DoF Tracking of a Passive Stylus__

Po-Chen's papers, thesis is quite helpful for understanding the theory behind the code. 

This is the website: http://media.ee.ntu.edu.tw/personal/pcwu/ . His thesis can be downloaded from here.

# Branch Convention
Please do not commit to master. The master should have fully working codes at all times.

Currently there is a dev branch. If you are adding new features, please commit them to the dev branch and raise pull requests to master. Also, before placing a pull request, please make sure your code is "merge-able" GitHub will let you know if there are issues.

You can start by cloning the dev branch with :

 ```git clone -b dev https://github.com/arkadeepnc/Visual-6-DoF-pose-tracker.git```


# REQUIREMENTS

UBUNTU 16.04 LTS [ not sure if it works with virtual Linux machine in Windows ]

ROS KINETIC

ros point_grey_camera_driver 

Read the Docs here: http://wiki.ros.org/pointgrey_camera_driver

Install prebuilt bins from source :

```sudo apt-get install ros-kinetic-pointgrey-camera-driver```

Connect camera to USB 3.1 (blue port). USB 2 would not work!! Start camera with : 

```roslaunch pointgrey_camera_driver camera.launch```

Check feed 
```rostopic echo /camera/image_color ```

should show a bunch of numbers in the screen within seconds after hitting return! Or else rostopic is dead. 

# Library dependencies:  
[all are basic and should be there if your Ubuntu-ROS do not hate each other]

numpy (latest)

OpenCV contrib (test it as with the following in the terminal: >python -> >import  cv2 -> >from cv2 import aruco)

rospy

matplotlib

transforms3d (pip install transforms3d)

scipy (check pypi for latest distro)

snakeviz -- for profiling your code


# Get started
Begin by running 

```python video_drawing_with_Dodeca.py ```

This will go through a video of the pen tip moving on a plane and generate the point cloud for the tip of the pen.

If you want to convert the code to C++ this might be a good starting point

https://github.com/neconeconeco/PressPen --> https://github.com/neconeconeco/PressPen/tree/master/Pen%20Tracking%20Algorithm

