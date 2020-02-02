#!/usr/bin/env python

import sys
import math
import json


import math
from geometry_msgs.msg import TransformStamped, Vector3
from aruco_msgs.msg import MarkerArray
import rospy
import tf2_ros

#Global marker
marker = None

#callback: invoked when the subscriber recieves a message
def marker_callback(msg):
    global marker
    #rospy.loginfo('New marker detected:\n%s', msg)
    marker = msg

#transform from camera_link to aruco detectedX
def transform_from_marker(m):
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = m.header.frame_id
    t.child_frame_id = "/aruco/detected" + str(m.id)
    t.transform.translation = Vector3(*[m.pose.pose.position.x, m.pose.pose.position.y
                                     , m.pose.pose.position.z])
    t.transform.rotation = m.pose.pose.orientation
    return t


#initiating first node and creating subscriber
rospy.init_node('fc1')
sub_aruco_markers = rospy.Subscriber('/aruco/markers', MarkerArray, marker_callback)
#Init dynamic broadcaster
brc = tf2_ros.TransformBroadcaster()

def main():
    rate = rospy.Rate(10) #Hz
    while not rospy.is_shutdown():
        if marker:
            for i in marker.markers:
                rospy.loginfo("Marker ID: " + str(i.id))
                trans_vec = [transform_from_marker(i)]
            #trans_vec = [transform_from_marker(m) for m in marker.markers]
            brc.sendTransform(trans_vec)

            rate.sleep()

if __name__ == "__main__":
    main()

