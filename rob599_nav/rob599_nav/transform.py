#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import tf2_geometry_msgs

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry


class TransformExample(Node):
	def __init__(self):
		super().__init__('transform_example')

		self.tf_buffer = Buffer()
		self.tf_listener = TransformListener(self.tf_buffer, self)

		self.sub = self.create_subscription(Odometry, 'odom', self.callback, 10)

	def callback(self, msg):
		point = PointStamped()
		point.header.frame_id = 'odom'
		point.point = msg.pose.pose.position

		self.get_logger().info(f'Odometry Pose: ({point.point.x:.3f}, {point.point.y:.3f}')

		try:
			new_point = self.tf_buffer.transform(point, 'map', rclpy.duration.Duration(seconds=1))
			self.get_logger().info(f'Map Pose: ({new_point.point.x:.3f}, {new_point.point.y:.3f}')
		except TransformException as e:
			self.get_logger().info(f'Transform failed: {e}')


def main(args=None):
	rclpy.init(args=args)

	tf_example = TransformExample()

	rclpy.spin(tf_example)

	rclpy.shutdown()


if __name__ == '__main__':
	main()