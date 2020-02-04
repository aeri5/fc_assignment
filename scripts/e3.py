#!/usr/bin/env python

import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



def camera_image_cb(ros_img):
	try:
		bgr_img = bridge.imgmsg_to_cv2(ros_img, "bgr8")     #image width 640, height 480
		#blur_img = cv2.GaussianBlur(bgr_img, (5,5), 0)
	except CvBridgeError as e:
		print(e)


	# Convert BGR to HSV
	hsv_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)

	# define range of color in HSV
	lower_limit = np.array([4, 50, 50])
	upper_limit = np.array([15, 255 ,255])

	# Threshold the HSV image to get colors in range
	bin_img = cv2.inRange(hsv_img, lower_limit, upper_limit)


	morph_img = cv2.morphologyEx(bin_img, cv2.MORPH_OPEN, np.ones((3,3)))

	contours, hierarchy = cv2.findContours(morph_img, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)   #RETR_EXTERNAL RETR_CCOMP

	try:
		hierarchy = hierarchy[0]

		nwalls = len([w for w in hierarchy if w[3] < 0])
		print("Found " + str(nwalls) + " walls with moments at:")

		for i,z in enumerate(zip(contours, hierarchy)):
			zip_c = z[0]
			zip_h = z[1]


			if zip_h[3] < 0:	#if no parents, contour is of hierarchy 1
				cv2.drawContours(bgr_img, zip_c, -1, (0, 0, 255), 2, cv2.LINE_AA)	#paint red

				M = cv2.moments(zip_c)
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])
				cv2.circle(bgr_img, (cx, cy), 2, (0, 0, 255), -1)


				print("(" + str(cx) + ", " + str(cy) + ")")


			elif zip_h[2] < 0:	#if no children, contour is of hierarchy 2
				cv2.drawContours(bgr_img, zip_c, -1, (0, 255, 0), 2, cv2.LINE_AA)	#paint green

				M = cv2.moments(zip_c)
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])
				cv2.circle(bgr_img, (cx, cy), 2, (0, 255, 0), -1)

				print("     with hole at (" + str(cx) + ", " + str(cy) + ")")

		print("\n\n")

	except TypeError as e:
		print("No walls detected")

	try:
		pub_bw_image.publish(bridge.cv2_to_imgmsg(bgr_img, "bgr8")) # mono8 bgr8
	except CvBridgeError as e:
		print(e)


rospy.init_node('e3')
sub_camera_image = rospy.Subscriber('/cf1/camera/image_raw', Image, camera_image_cb)
pub_bw_image = rospy.Publisher('/e3_image', Image, queue_size=2)
bridge = CvBridge()




def main():
	rate = rospy.Rate(20)
	print("running")
	while not rospy.is_shutdown():
		rate.sleep()

if __name__ == '__main__':
	main()