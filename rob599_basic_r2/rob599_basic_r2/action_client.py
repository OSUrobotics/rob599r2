#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from rob599_interfaces.action import Fibonacci


class FibonacciClient(Node):
	def __init__(self):
		super().__init__('fib_client')

		self.client = ActionClient(self, Fibonacci, 'fibonacci')

	def send_goal(self, n):
		goal = Fibonacci.Goal()
		goal.number = n

		self.client.wait_for_server()

		self.result = self.client.send_goal_async(goal, feedback_callback=self.feedback)
		self.result.add_done_callback(self.done)

	def feedback(self, feedback):
		pass

	def done(self, result):
		goal = result.result()

		if not goal.accepted:
			self.get_logger().info('Goal rejected')
			return

		result_handle = goal.get_result_async()
		result_handle.add_done_callback(self.process_result)

	def process_result(self, future):
		result = future.result().result

		self.get_logger().info('Result: {0}'.format(list(result.sequence)))


def main(args=None):
	rclpy.init(args=args)

	client = FibonacciClient()

	client.send_goal(10)

	rclpy.spin(client)


if __name__ == '__main__':
	main()
