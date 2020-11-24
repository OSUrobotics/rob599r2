#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64


class BasicSubscriber(Node):
	def __init__(self):
		super().__init__('subscriber')
		self.sub = self.create_subscription(Int64, 'counter', self.callback, 10)

	def callback(self, msg):
		self.get_logger().info('Got {0}'.format(msg.data))


def main(args=None):
	rclpy.init(args=args)

	subscriber = BasicSubscriber()

	rclpy.spin(subscriber)
	rclpy.shutdown()
	

if __name__ == '__main__':
	main()

