#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan


class FilterPublisher(Node):
	def __init__(self):
		super().__init__('filter_publisher')

		self.pub_1 = self.create_publisher(LaserScan, 'one', 10)
		self.pub_2 = self.create_publisher(LaserScan, 'two', 10)

		self.timer_1 = self.create_timer(1, self.callback_1)
		self.timer_2 = self.create_timer(2, self.callback_2)

	def callback_1(self):
		laser = LaserScan()
		laser.header.stamp = self.get_clock().now().to_msg()

		self.get_logger().info('Publishing for Laser 1')
		self.pub_1.publish(laser)


	def callback_2(self):
		laser = LaserScan()
		laser.header.stamp = self.get_clock().now().to_msg()

		self.get_logger().info('Publishing for Laser 2')
		self.pub_2.publish(laser)


def main(args=None):
	rclpy.init(args=args)

	publisher = FilterPublisher()

	rclpy.spin(publisher)

	rclpy.shutdown()


if __name__ == '__main__':
	main()
