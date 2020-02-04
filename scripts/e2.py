#!/usr/bin/env python

import math
import rospy
import tf2_ros
import tf2_geometry_msgs
from collections import deque
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Vector3
from crazyflie_driver.msg import Position

checkpoints = deque()
first_t = None

def currentpos_cb(c):
	global first_t
	d2c = math.sqrt((c.pose.position.x - checkpoints[0].pose.position.x)**2 + (c.pose.position.y - checkpoints[0].pose.position.y)**2 + (c.pose.position.z - checkpoints[0].pose.position.z)**2)
	print("distance to checkpoint %s", d2c)

	if d2c <= 0.15:
		if not first_t:
			first_t = rospy.Time.now()
		else:
			if c.header.stamp.secs-first_t.secs >= 2:
				checkpoints.popleft()
				first_t = None



def create_checkpoints():
	global checkpoints

	#Create goal poses
	g = PoseStamped()
	g.header.frame_id = 'map'
	g.pose.position.x = 0.0
	g.pose.position.y = 0.0
	g.pose.position.z = 0.4
	(g.pose.orientation.x, g.pose.orientation.y, g.pose.orientation.z, g.pose.orientation.w) = quaternion_from_euler(math.radians(0), math.radians(0), math.radians(-90)) #(roll, pitch, yaw)
	checkpoints.append(g)

	g = PoseStamped()
	g.header.frame_id = 'map'
	g.pose.position.x = 0.0
	g.pose.position.y = -4.5
	g.pose.position.z = 0.4
	(g.pose.orientation.x, g.pose.orientation.y, g.pose.orientation.z, g.pose.orientation.w) = quaternion_from_euler(math.radians(0), math.radians(0), math.radians(-90)) #(roll, pitch, yaw)
	checkpoints.append(g)

	g = PoseStamped()
	g.header.frame_id = 'map'	
	g.pose.position.x = 2
	g.pose.position.y = -2.0
	g.pose.position.z = 0.4
	(g.pose.orientation.x, g.pose.orientation.y, g.pose.orientation.z, g.pose.orientation.w) = quaternion_from_euler(math.radians(0), math.radians(0), math.radians(0)) #(roll, pitch, yaw)
	checkpoints.append(g)


	g = PoseStamped()
	g.header.frame_id = 'map'
	g.pose.position.x = 0.0
	g.pose.position.y = 1.5
	g.pose.position.z = 1.5
	(g.pose.orientation.x, g.pose.orientation.y, g.pose.orientation.z, g.pose.orientation.w) = quaternion_from_euler(math.radians(0), math.radians(0), math.radians(270)) #(roll, pitch, yaw)
	checkpoints.append(g)


	g = PoseStamped()
	g.header.frame_id = 'map'
	g.pose.position.x = -3.0
	g.pose.position.y = 0.0
	g.pose.position.z = 1
	checkpoints.append(g)

	print("Created checkpoints:\n" + str(checkpoints))


def publish_checkpoint(c):
	c.header.stamp = rospy.Time.now()

	if not tf_buf.can_transform(c.header.frame_id, 'cf1/odom', c.header.stamp):
		rospy.logwarn_throttle(5.0, 'No transform from %s to cf1/odom' % c.header.frame_id)
		return

	c_odom = tf_buf.transform(c, 'cf1/odom')

	c_out = Position()
	c_out.header.stamp = rospy.Time.now()
	c_out.header.frame_id = c_odom.header.frame_id
	c_out.x = c_odom.pose.position.x
	c_out.y = c_odom.pose.position.y
	c_out.z = c_odom.pose.position.z
	roll, pitch, yaw = euler_from_quaternion((c_odom.pose.orientation.x, c_odom.pose.orientation.y, c_odom.pose.orientation.z, c_odom.pose.orientation.w))
	c_out.yaw = math.degrees(yaw)

	pub_checkpoints.publish(c_out)



rospy.init_node('e2')
pub_checkpoints  = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
sub_current_pos = rospy.Subscriber('/cf1/pose', PoseStamped, currentpos_cb)

tf_buf   = tf2_ros.Buffer()
tf_lstn  = tf2_ros.TransformListener(tf_buf)

def main():
	rate = rospy.Rate(20)  # Hz
	
	while not rospy.is_shutdown():
		if len(checkpoints) > 0:
			publish_checkpoint(checkpoints[0])
		else:
			create_checkpoints()
		rate.sleep()

if __name__ == '__main__':
	main()