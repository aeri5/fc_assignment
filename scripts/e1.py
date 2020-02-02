#!/usr/bin/env python

import rospy, tf2_ros
from geometry_msgs.msg import TransformStamped, Vector3
from aruco_msgs.msg import MarkerArray

# init marker var
markers = None


def markers_callback(msg):
    global markers
    markers = msg.markers

    rospy.loginfo("Found %s aruco markers with id: %s", len(markers), [m.id for m in markers])


def greta(m):
    t = TransformStamped()
    t.header.stamp = m.header.stamp
    t.header.frame_id = '/cf1/camera_link' #m.header.frame_id
    t.child_frame_id = '/aruco/detected' + str(m.id)

    t.transform.translation = Vector3(*[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z])
    t.transform.rotation = m.pose.pose.orientation

    return t
    
rospy.init_node('e1')
sub_markers = rospy.Subscriber('/aruco/markers', MarkerArray, markers_callback)
tf_broadcaster = tf2_ros.TransformBroadcaster()


def main():
    rate = rospy.Rate(20)  # Hz
    while not rospy.is_shutdown():
        print(".")
        if markers:
            transforms = [greta(m) for m in markers]
            tf_broadcaster.sendTransform(transforms)
        rate.sleep()

if __name__ == '__main__':
    main()