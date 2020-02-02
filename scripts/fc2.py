#!/usr/bin/env python

import math
import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from crazyflie_driver.msg import Position
import tf2_ros
import tf2_geometry_msgs
from crazyflie_driver.msg import Position


#init command goal
cmdgoal = None
poses_list = []
i = 0

#current
def cmdgoal_callback(msg):
    global cmdgoal
    cmdgoal = msg

def publish_cmd(cmdgoal):
    print(cmdgoal)
    cmdgoal.header.stamp = rospy.Time.now()

    #transform from map to odom, you have the transformation in the buffer
    cmdgoal_odom = buffer.transform(cmdgoal, 'cf1/odom')

    #create a new message of type Position(), with cmdgoal, to publish
    cmd_to_pub = Position()
    cmd_to_pub.x, cmd_to_pub.y, cmd_to_pub.z =   cmdgoal_odom.pose.position.x,cmdgoal_odom.pose.position.y, cmdgoal_odom.pose.position.z
    
    #the angles are given in quaternion, convert to Euler
    roll, pitch, yaw = euler_from_quaternion((cmdgoal_odom.pose.orientation.x, cmdgoal_odom.pose.orientation.y,cmdgoal_odom.pose.orientation.z, cmdgoal_odom.pose.orientation.x))

    cmd_to_pub.yaw = yaw

    pub_cmdgoal.publish(cmd_to_pub)
    
rospy.init_node('fc2')
subs_cmdgoal = rospy.Subscriber('/cf1/pose', PoseStamped, cmdgoal_callback)
pub_cmdgoal = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)

buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(buffer)

def generate_cmdgoal(cmdgoal_list):
    global poses_list
    for poses in cmdgoal_list:
        poses_list.append(PoseStamped())
        for element in poses_list:
            element.header.frame_id = 'map'
            element.pose.position.x = poses[0]
            element.pose.position.y = poses[1]
            element.pose.position.z = poses[2]

            

def main():
    cmdgoal_list = [(0, 0 ,0.4), (-1, 2, 0.4), (-1.5, 2, 0.3)]

    generate_cmdgoal(cmdgoal_list)
    rate = rospy.Rate(10) #Hz
    while not rospy.is_shutdown():
        if poses_list:
            cmdgoal = poses_list[i]
            publish_cmd(cmdgoal)
        rate.sleep()

if __name__ == '__main__':
    main()

