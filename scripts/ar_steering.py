#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion
from ros2_aruco_interfaces.msg import ArucoMarkers


MAX_LINEAR_SPEED = 0.1
MAX_ANGULAR_SPEED = 0.5

def linear_fit(input, max_input, max_output):
	input = max(-max_input, min(input, max_input))

	return (input / max_input) * max_output


class ArSteeringNode(Node):
	def __init__(self):
		super().__init__('ar_steering')

		self.declare_parameter('marker_id', 0)
		self.declare_parameter('cmd_vel_topic', '/cmd_vel')

		self.marker_id = self.get_parameter('marker_id').value
		cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

		self.get_logger().info(f"Marker ID: {self.marker_id}")
		self.get_logger().info(f"Velocity command topic {cmd_vel_topic}")

		self.sub_poses = self.create_subscription(ArucoMarkers, '/aruco_markers', self.pose_callback, 1)
		self.pub_twist = self.create_publisher(Twist, cmd_vel_topic, 1)


	def pose_callback(self, msg):
		cmd = Twist()

		# Find the desired marker, otherwise publish ZERO speed
		if self.marker_id in msg.marker_ids:
			marker_idx = msg.marker_ids.index(self.marker_id)
			pose = msg.poses[marker_idx]
			orientation = pose.orientation

			roll, _pitch, yaw = euler_from_quaternion([
				orientation.x,
				orientation.y,
				orientation.z,
				orientation.w
			])

			if roll > -2.9 and roll < 2.9:
				roll = math.atan2(math.sin(roll - math.pi), math.cos(roll - math.pi))
				cmd.linear.x = linear_fit(roll, 1.0, MAX_LINEAR_SPEED)
			if yaw < -0.1 or yaw > 0.1:
				cmd.angular.z = linear_fit(yaw, 1.0, MAX_ANGULAR_SPEED)

		self.pub_twist.publish(cmd)


def main(args=None):
	rclpy.init(args=args)
	node = ArSteeringNode()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
