''' Plot the point cloud from generated txt file'''

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import matplotlib.pyplot as plt
from matplotlib.mlab import griddata
import numpy as np
from math import log
import matplotlib
matplotlib.use('GTKAgg') 
 

 
data = np.loadtxt('gamepad')

y = data[:,1]
x = data[:,0]
z = data[:,2]

fig = plt.figure()

ax = fig.add_subplot(111, projection="3d")
fig.canvas.set_window_title("translation x,y,z") 

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

ax.scatter(x,y,z,c ='m',label = "pose_marker_without_opt")

plt.show()




