#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from rob599_interfaces.srv import Doubler


class BasicServiceClient(Node):
	def __init__(self):
		super().__init__('client')

		self.client = self.create_client(Doubler, 'doubler')

		while not self.client.wait_for_service(timeout_sec=1):
			self.get_logger().info('waiting for service to start')

	def send_request(self, number):
		request = Doubler.Request()
		request.number = number

		self.response = self.client.call_async(request)


def main(args=None):	
	rclpy.init(args=args)

	client = BasicServiceClient()

	for i in range(5):
		client.send_request(i)

		while rclpy.ok():
			rclpy.spin_once(client)

			if client.response.done():
				try:
					answer = client.response.result()
				except Exception as e:
					client.get_logger().info('Service call failed: {0}'.format(e))
				else:
					client.get_logger().info('Send {0}, got {1}'.format(i, answer.doubled))
					break;

	rclpy.shutdown()


if __name__ == '__main__':
	main()
