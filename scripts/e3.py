#!/usr/bin/env python

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



def camera_image_cb(ros_img):
	try:
		cv_img = bridge.imgmsg_to_cv2(ros_img, "mono8")
	except CvBridgeError as e:
		print(e)

	#cv_img = cv2.medianBlur(cv_img,5) 
	cv_img = cv2.GaussianBlur(cv_img,(1,1),0)

	cv_img = cv2.adaptiveThreshold(cv_img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,2)
	#r, cv_img = cv2.threshold(cv_img, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)

	try:
		pub_bw_image.publish(bridge.cv2_to_imgmsg(cv_img, "mono8"))
	except CvBridgeError as e:
		print(e)










rospy.init_node('e3')
sub_camera_image = rospy.Subscriber('/cf1/camera/image_raw', Image, camera_image_cb)
pub_bw_image = rospy.Publisher('/e3_image', Image, queue_size=2)
bridge = CvBridge()




def main():
	rate = rospy.Rate(20)

	while not rospy.is_shutdown():
		print("hej")
		rate.sleep()

if __name__ == '__main__':
	main()