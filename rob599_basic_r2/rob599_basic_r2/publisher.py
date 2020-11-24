#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64


class BasicPublisher(Node):
	def __init__(self):
		super().__init__('publisher')
		self.pub = self.create_publisher(Int64, 'counter', 10)

		self.timer = self.create_timer(1, self.callback)
		self.counter = 0

	def callback(self):
		msg = Int64()
		msg.data = self.counter

		self.pub.publish(msg)

		self.get_logger().info('Published {0}'.format(self.counter))
		self.counter += 1


def main(args=None):
	rclpy.init(args=args)

	publisher = BasicPublisher()

	rclpy.spin(publisher)
	rclpy.shutdown()


if __name__ == '__main__':
	main()