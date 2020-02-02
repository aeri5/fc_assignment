#!/usr/bin/env python

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import roslib
from std_msgs.msg import String
import numpy as np
import rospy

def image_callback(img_msg):
    #convert image from ROS format to CV
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, 'mono8')
    except CvBridgeError as e:
        print(e)
        
    cv_image = cv2.GaussianBlur(cv_image, (5,5),  0)
    cv_image = cv2.adaptiveThreshold(cv_image,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,2)

    try:
        image_pub.publish(bridge.cv2_to_imgmsg(cv_image, 'mono8'))
    except CvBridgeError as e:
        print(e)


rospy.init_node('fc3')
image_pub = rospy.Publisher('/myresult', Image, queue_size=2)
image_sub = rospy.Subscriber('/cf1/camera/image_raw', Image, image_callback)

def main():
    rate = rospy.Rate(10) #Hz
    while not rospy.is_shutdown():
        print('tjena')
        rate.sleep()

if __name__ == '__main__':
    main()