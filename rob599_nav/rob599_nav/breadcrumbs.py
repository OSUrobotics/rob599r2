#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3, Pose, PoseStamped
from visualization_msgs.msg import Marker

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import tf2_geometry_msgs


class Breadcrumbs(Node):
	def __init__(self):
		super().__init__('breadcrumbs')

		self.tf_buffer = Buffer()
		self.tf_listener = TransformListener(self.tf_buffer, self)

		self.counter = 0

		self.pub = self.create_publisher(Marker, 'breadcrumbs', 10)

		self.timer = self.create_timer(1, self.callback)

	def callback(self):
		try:
			marker = self.make_breadcrumb()
			self.pub.publish(marker)
			self.get_logger().info(f'Dropped breadcrumb {marker.id}.')
		except TransformException as e:
			self.get_logger().info(f'Transform failed: {e}')

	def make_breadcrumb(self):
		# Make the marker.
		marker = Marker()

		marker.header.frame_id = 'map'
		marker.header.stamp = self.get_clock().now().to_msg()

		marker.id = self.counter
		marker.type = Marker.SPHERE
		marker.action = Marker.ADD

		# Set the size of the sphere.  It can be oblate, so we set three scales.
		marker.scale.x = 0.1
		marker.scale.y = 0.1
		marker.scale.z = 0.1

		# Set the color.
		marker.color.r = 0.0
		marker.color.g = 1.0
		marker.color.b = 0.0
		marker.color.a = 1.0

		# Get the current pose of the robot, in the map frame.
		marker.pose = self.get_current_pose(marker.header.frame_id)

		self.counter += 1

		return marker

	def get_current_pose(self, frame_id):
		# Build a stamped pose for the base link origin.  If we don't set the time in the header,
		# then we get the latest TF update.
		origin = PoseStamped()
		origin.header.frame_id = 'base_link'

		# Set the position.
		origin.pose.position.x = 0.0
		origin.pose.position.y = 0.0
		origin.pose.position.z = 0.0

		# Set an arbitrary orientation.
		origin.pose.orientation.x = 0.0
		origin.pose.orientation.y = 0.0
		origin.pose.orientation.z = 0.0
		origin.pose.orientation.w = 1.0

		# Get the transform to the map frame.  This will cause an exception if it fails, but we'll
		# deal with that in the calling function.
		new_pose = self.tf_buffer.transform(origin, frame_id, rclpy.duration.Duration(seconds=1))

		return new_pose.pose


def main(args=None):
	rclpy.init(args=args)

	breadcrumbs = Breadcrumbs()

	rclpy.spin(breadcrumbs)

	rclpy.shutdown()


if __name__ == '__main__':
	main()
