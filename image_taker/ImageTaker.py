 

from __future__ import division
 
import cv2
import time
import rospy
from roscam import RosCam
 

img_counter=52
 
rospy.init_node('RosCam', anonymous=True)


ic = RosCam("/camera/image_color")
while(True):
	frame = ic.cv_image
	if frame is None:
		time.sleep(0.1)
		print("No image")
		continue
	else:	
		cv2.imshow("test", frame)
 
		k = cv2.waitKey(1)

		if k%256 == 27:
			# ESC pressed
			print("Escape hit, closing...")
			break
		elif k%256 == 32:
			# SPACE pressed
			img_name = "opencv_frame_{}.jpg".format(img_counter)
			cv2.imwrite(img_name, frame)
			print("{} written!".format(img_name))
			img_counter += 1


cv2.destroyAllWindows()
