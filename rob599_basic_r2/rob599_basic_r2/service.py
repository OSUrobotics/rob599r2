#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

from rob599_interfaces.srv import Doubler


class BasicService(Node):
	def __init__(self):
		super().__init__('service')

		self.service = self.create_service(Doubler, 'doubler', self.callback)

	def callback(self, request, response):
		response.doubled = request.number * 2

		self.get_logger().info('Got {0}'.format(request.number))

		return response


def main(args=None):
	rclpy.init(args=args)

	service = BasicService()

	rclpy.spin(service)
	rclpy.shutdown()


if __name__ == '__main__':
	main()
