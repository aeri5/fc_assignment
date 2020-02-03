#!/usr/bin/env python
""" 
Author: Luis Alejandro Sarmiento Gonzalez
Mail: lasg@kth.se
Group 5
Task 3
"""
from __future__ import print_function
import rospy 
import sys 
import tf
import roslib
import cv2 
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


# We are going to create a class for object detection using thresholding (binarization), 
# and contour detection to approximate to a known basic figure. 
 
class ObjectDetection:
    
  def __init__(self):
    self.image_pub = rospy.Publisher("/myresult", Image, queue_size=2)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/cf1/camera/image_raw", Image, self.callback)

  def callback(self,data):
    # Convert the image from OpenCV to ROS format
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    """
      Fancy code here.
    """       
    # We convert to grayscale 
    gray_img=cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
    # Blur image
    blur_img=cv2.GaussianBlur(gray_img,(5,5),cv2.BORDER_DEFAULT)
    # Optimal thresholding with Otsu's method
    h,thresh_img=cv2.threshold(blur_img,50,255,-16-cv2.THRESH_OTSU)
    # Morphological operations for easier box detection.
    thresh_img = cv2.erode(thresh_img, None, iterations=6)#8
    thresh_img = cv2.dilate(thresh_img, None, iterations=5)#5
    # We just do AND operation between thresholded image and real image.
    th=cv_image-cv2.bitwise_and(cv_image,cv_image, mask= thresh_img)
    # We find contours
    _, contours, _ = cv2.findContours(thresh_img.copy(), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

    i=0
    #For each contour...
    for cnt in contours:
        # Focus on small contours, as signs will be small in simulation, saves a lot of false positives.
        if len(cnt)<500:
            # We calculate the moments of the contours, thus we can get the center of mass.
            moments=cv2.moments(cnt)
            if moments["m00"] !=0:
                cx=int(moments["m10"]/moments["m00"])
                cy=int(moments["m01"]/moments["m00"])
                #We can get the corners of the detected objects... we approximate a polygon given the contour.
                epsilon = 0.1*cv2.arcLength(cnt,True)
                approx = cv2.approxPolyDP(cnt,epsilon,True)
                #Draw results.
                cv2.drawContours(cv_image, contours, -1, (255,255,255), 1, cv2.LINE_AA)
                cv2.drawContours(cv_image, approx, -1, (255,0,0), 3, cv2.LINE_AA) 
                cv2.drawContours(th, approx, -1, (255,0,0), 3, cv2.LINE_AA)
                # Uncomment below if you want to add the number of detected objects, disabled as of now due to a bug in OpenCV 3.2... prints an annoying line.
                #cv2.putText(cv_image,str(i), (cx, cy), cv2.FONT_HERSHEY_SIMPLEX,0.3, (0, 0, 255), 2)
                i=i+1
                #Position of the detected object in the image.
                print('Object Detected with center at picture coordinate: (%i,%i)'%(cx,cy))

    cv2.imshow("Image Window",th)
    cv2.waitKey(1)
        
    #For pose estimation of the objects in the world, like with aruco, it can be done with  solvePnPRansac.
    #The function estimates an object pose given a set of object points, their corresponding image projections, as well as the camera matrix and the distortion coefficients. 
    
    """
    Fancy code finishes here.
    """
    # Publish the image
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image,"bgr8"))#np.asarray(thresh_img),"mono8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  rospy.init_node('imagecircle', anonymous=True)

  ic = ObjectDetection()

  print("running...")
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)