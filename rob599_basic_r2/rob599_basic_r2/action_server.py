#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from rob599_interfaces.action import Fibonacci

from std_msgs.msg import Int64


# We're going to use this as the main worker function for the action server.  We've intentionally
# implemented this naively, so that it takes a long time to run for larger numbers.
def fibonacci(n):
	"""
	A naive implementation of Fibonacci numbers.

	:param n: An integer.
	:return: The nth Fibonacci number.
	"""
	if n < 2:
		return n
	else:
		return fibonacci(n - 1) + fibonacci(n - 2)


class FibonacciActionServer(Node):
	def __init__(self):
		super().__init__('fibber')

		self.server = ActionServer(self, Fibonacci, 'fibonacci', self.callback)

	def callback(self, goal):
		self.get_logger().info('Got {0}'.format(goal.request.number))

		result = Fibonacci.Result()
		result.sequence = []
		for i in range(goal.request.number + 1):
			result.sequence.append(fibonacci(i))

		goal.succeed()
		self.get_logger().info('Result: {0}'.format(result.sequence))
		return result


def main(args=None):
	rclpy.init(args=args)

	server = FibonacciActionServer()

	rclpy.spin(server)


if __name__ == '__main__':
	main()
