#!/usr/bin/env python

import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



def camera_image_cb(ros_img):
	img_clr = bridge.imgmsg_to_cv2(ros_img, "bgr8")
	img_gray = bridge.imgmsg_to_cv2(ros_img, "mono8")

	img_blur = cv2.GaussianBlur(img_gray, (11,11), .001)

	img_thresh = cv2.adaptiveThreshold(img_blur,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,2)

	img_morph = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, np.ones((7,7)))

	contours, hierarchy = cv2.findContours(img_morph, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
	
	for contour in contours:
		if cv2.arcLength(contour,True)<1000 and cv2.arcLength(contour,True)>20:
			x,y,w,h = cv2.boundingRect(contour)
			img_clr = cv2.rectangle(img_clr,(x,y),(x+w,y+h),(0,255,0),2)


		#get moments and print center points of detected objects





	try:
		pub_bw_image.publish(bridge.cv2_to_imgmsg(img_clr, "bgr8")) # mono8 bgr8
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