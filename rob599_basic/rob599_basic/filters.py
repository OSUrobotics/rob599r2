#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
import message_filters

from sensor_msgs.msg import LaserScan


class FilterExample(Node):
	def __init__(self):
		super().__init__('filter_example')

		self.sub1 = message_filters.Subscriber(self, LaserScan, 'one')
		self.sub2 = message_filters.Subscriber(self, LaserScan, 'two')

		# Set up a time synchronizer for both publishers, queue size of 10, with a time window of 0.01s.
		self.ts = message_filters.ApproximateTimeSynchronizer([self.sub1, self.sub2], 10, 0.01)
		self.ts.registerCallback(self.callback)

	def callback(self, msg_1, msg_2):
		self.get_logger().info(f'Got a pair of scans: {msg_1.header.stamp} and {msg_2.header.stamp}')


def main(args=None):
	rclpy.init(args=args)

	filter = FilterExample()

	rclpy.spin(filter)

	rclpy.shutdown()


if __name__ == '__main__':
	main()
