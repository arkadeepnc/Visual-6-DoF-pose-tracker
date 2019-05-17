import numpy as np
import cv2 as cv
import glob
# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER,22, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((9*6,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
objp = objp.reshape(-1,1,3)*22.74
print objp
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('*.jpg')
i = 0
for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    print fname
  # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (9,6), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (12,12), (-1,-1), criteria)
        imgpoints.append(corners)
        Draw and display the corners
        cv.drawChessboardCorners(img, (9,6), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(0)
        i = i+1
        print i

        # cv.destroyAllWindows()

print "thand rakhiye, calibrate ho raha hain"     
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)


print "ho gaya"
cv.imwrite('CALIBRATION DEMO.png',img)
img = cv.imread('opencv_frame_16.jpg')
h,  w = img.shape[:2]

newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))


print newcameramtx


# # undistort
# dst = cv.undistort(img, mtx, dist, None, newcameramtx)

# # crop the image
# x,y,w,h = roi
# dst = dst[y:y+h, x:x+w]
# cv.imwrite('calibresult.png',dst)
# # cv.imshow('calibresult.png')
# img = cv.imread('calibresult.png')
# cv.imshow('image',img)
# cv.waitKey(0)

# np.savez ("B", mtx = newcameramtx, dist = dist, rvecs =rvecs, tvecs =tvecs)

# mean_error = 0
# for i in xrange(len(objpoints)):
#     imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
#     error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
#     mean_error += error
# print( "total error: {}".format(mean_error/len(objpoints)) )


 # np.load
img = cv.imread("opencv_frame_4.jpg")
ret, corners = cv.findChessboardCorners(img, (9,6), None)
print corners[1], corners[2] 