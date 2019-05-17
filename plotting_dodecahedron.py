''' python3 code for dodecahedron'''
# #!/usr/bin/env python3
import itertools as it
import numpy as np
from numpy import linalg as LA
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.pyplot as plt
# from tempfile import TemporaryFile
scale_fac = 16.18033988749895  # default is 1, scale it to make equal to your pentagon side
v=[p for p in it.product((-1,1),(-1,1),(-1,1))]
g=.5+.5*5**.5
v.extend([p for p in it.product((0,),(-1/g,1/g),(-g,g))])
v.extend([p for p in it.product((-1/g,1/g),(-g,g),(0,))])
v.extend([p for p in it.product((-g,g),(0,),(-1/g,1/g))])
v=np.array(v)
g=[[12,14,5,9,1],\
    [12,1,17,16,0],\
    [12,0,8,4,14],\
    [4,18,19,5,14],\
    [4,8,10,6,18],\
    [5,19,7,11,9],\
    [7,15,13,3,11],\
    [7,19,18,6,15],\
    [6,10,2,13,15],\
    [13,2,16,17,3],\
    [3,17,1,9,11],\
    [16,2,10,8,0]]

# g = [[12,0,8,4,14],\
#      [7,19,18,6,15],\
#      [6,10,2,13,15],\
#      [12,1,17,16,0],\
#      [13,2,16,17,3],\
#      [5,19,7,11,9],\
#      [4,8,10,6,18],\
#      [12,14,5,9,1],\
#      [5,19,7,11,9],\
#      [7,15,13,3,11],\
#      [4,18,19,5,14],\
#      [3,17,1,9,11]]

a=[2,1,0,3,4,5,0,1,2,3,4,5]
fig = plt.figure()
ax = fig.add_subplot((111),aspect='equal',projection='3d')
# ax = fig.gca(projection='3d')

Scl_mat = [[scale_fac,0, 0],[0, scale_fac, 0],[0, 0, scale_fac]] ;

for i in range(20):
    v[i,:] = np.matmul(Scl_mat,v[i,:])
"if required, we can transform here"

"for plotting the frame at the origin"
uo=np.array([1,0,0])  # x axis
vo=np.array([0,1,0])  # x axis
wo=np.array([0,0,1])  # x axis

ax.set_xlim3d(-2*scale_fac, 2*scale_fac)
ax.set_ylim3d(-2*scale_fac, 2*scale_fac)
ax.set_zlim3d(-2*scale_fac, 2*scale_fac)
#ax.hold(True)

# ax.quiver(0, 0, 0, uo, vo, wo, length=1, normalize=True)
ax.quiver(0, 0, 0,uo[0], uo[1],uo[2],color ='r' ,length=20, normalize=True)
ax.quiver(0, 0, 0,vo[0], vo[1],vo[2], color ='b',length=20, normalize=True)
ax.quiver(0, 0, 0,wo[0], wo[1],wo[2], color = 'g',length=20, normalize=True)




face_markers = ['1','2','3','4','5','6','7','8','9','10','11','12']

def getMarker(i):
    # Use modulus in order not to have the index exceeding the lenght of the list (markers)
    return "$"+face_markers[i]+"$"

def get_face_frame(vertices):
    "input is a 5 x n array of vertex points"
    x_ax = vertices[0,:] - vertices[2,:] 
    x_ax = x_ax/LA.norm(x_ax,2)
    z_ax = np.cross(vertices[0,:] - vertices[1,:], vertices[0,:] - vertices[3,:])
    z_ax = z_ax/LA.norm(z_ax,2)
    y_ax = np.cross (z_ax, x_ax)
    
    x_ax = np.reshape(x_ax,(3,1))
    y_ax = np.reshape(y_ax,(3,1))
    z_ax = np.reshape(z_ax,(3,1))
    
    # print(x_ax.shape)
    
    r_mat= np.hstack((x_ax,y_ax,z_ax))
    
    face_center = np.array([np.mean(vertices[:,0]), np.mean(vertices[:,1]), np.mean(vertices[:,2])])
    face_center = np.reshape(face_center,(3,1))
    
    return face_center, r_mat

arr_colors = ['r'] #,'g','b']

# c=Poly3DCollection([[tuple(y) for y in v[g[f],:]]], linewidths=1, alpha=1)
# c.set_facecolor([(0,0,1),(0,1,0),(0,1,1),(1,0,0),(1,0,1),(1,1,0)][a[f]])
# ax.add_collection3d(c)

# faces = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
faces =   [3, 8, 9, 2, 10, 12, 5, 1, 6, 7, 4, 11]

R_cent_face = np.zeros((12,3,3))
T_cent_face = np.zeros((12,3,1))

for f in range(12):
    # f-=1
    y = v[g[f],:]
    ax.scatter (y[:,0], y[:,1], y[:,2])
    ax.plot(y[:,0], y[:,1], y[:,2],color='r')
    ax.scatter (np.mean(y[:,0]), np.mean(y[:,1]), np.mean(y[:,2]), marker=getMarker(f) , s=180)
    (T_cent_face[f,:,:], R_cent_face[f,:,:] ) = get_face_frame(y)
    (face_center, r_mat ) = get_face_frame(y)
    r_mat = r_mat.T
    ax.quiver(face_center[0], face_center[1], face_center[2], r_mat[0,0], r_mat[0,1], r_mat[0,2],color='r', length=10, normalize=True) # x axis
    ax.quiver(face_center[0], face_center[1], face_center[2], r_mat[1,0], r_mat[1,1], r_mat[1,2],color='b', length=10, normalize=True) # y axis
    ax.quiver(face_center[0], face_center[1], face_center[2], r_mat[2,0], r_mat[2,1], r_mat[2,2],color='g', length=10, normalize=True) # z axis

    # ax.quiver(T_cent_face[f,1,0],T_cent_face[f,1,0],T_cent_face[f,2,0], R_cent_face[f,0,0], R_cent_face[f,1,0],R_cent_face[f,2,0],color='r', length=0.3, normalize=True) # x axis
    # ax.quiver(T_cent_face[f,1,0],T_cent_face[f,1,0],T_cent_face[f,2,0], R_cent_face[f,0,1], R_cent_face[f,1,1],R_cent_face[f,2,1],color='r', length=0.3, normalize=True) # x axis
    # ax.quiver(T_cent_face[f,1,0],T_cent_face[f,1,0],T_cent_face[f,2,0], R_cent_face[f,0,2], R_cent_face[f,1,2],R_cent_face[f,2,2],color='r', length=0.3, normalize=True) # x axis

ax.auto_scale_xyz

np.save('Center_face_rotations.npy',R_cent_face)
np.save('Center_face_translations.npy',T_cent_face)

face_id = 2
x_origin=np.array([[1],[1],[0],[1]])
T_cent_face_curr = T_cent_face[face_id-1,:,:]
R_cent_face_curr = R_cent_face[face_id-1,:,:]
T_mat_cent_face = np.vstack((np.hstack((R_cent_face_curr,T_cent_face_curr)),np.array([0,0,0,1])))
transformed_x = np.matmul(T_mat_cent_face, x_origin)
cent = T_cent_face_curr 
# rmat = R_cent_face_curr
# ax.quiver(cent[0], cent[1], cent[2], transformed_x[0], transformed_x[1], transformed_x[2],color='k', length=20, normalize=True) # x axis
"Now we draw a sphere"
#
#r_circumscribe = scale_fac*np.sqrt(3)/4*(1+np.sqrt(5))
##r_inscribe = 0.5*scale_fac*np.sqrt(5/2 + 11/10 * np.sqrt(5))
#
#u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
#
#xs = np.cos(u)*np.sin(v)*r_circumscribe
#ys = np.sin(u)*np.sin(v)*r_circumscribe
#zs = np.cos(v)*r_circumscribe
#
#ax.plot_wireframe(xs, ys, zs, color="r")

plt.show()


#def plot_frame (u,v,w):
    
