#!/usr/bin/env python3

# Example of an action server in ROS 2
#
# action_server.py
#
# Bill Smart
#
# This is an example of a action server in ROS 2.


# Pull in the stuff we need from rclpy.import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

# Pull in the action definition.  As with other compound messages, we only need to bring
# in the base type, since all of the other types are defined inside of it.
from rob599_msgs.action import Fibonacci

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


# The idiom in ROS2 is to use a function to do all of the setup and work.  This
# function is referenced in the setup.py file as the entry point of the node when
# we're running the node with ros2 run.  The function should have one argument, for
# passing command line arguments, and it should default to None.
class FibonacciActionServer(Node):
	def __init__(self):
		# Initialize the superclass
		super().__init__('fibber')

		# Set up a simple action server.  This is a different procedure than for topics
		# and services, since actions are still not really first-class citizens.  However,
		# we're still going to pass in the node (as self), in addition to a type, an
		# action name, and a callback.
		self.server = ActionServer(self, Fibonacci, 'fibonacci', self.callback)

	# This is the callback the services the action request.
	def callback(self, goal):
		# Grab the logger and send a message to it.
		self.get_logger().info('Got {0}'.format(goal.request.number))

		# Build a result to send the sequence to.  We can do this through the base
		# action type.  Once we build it, we set the return data, in sequence, to an
		# empty list.
		result = Fibonacci.Result()
		result.sequence = []

		# Incrementally fill in the elements of the list by making calls to the naive
		# Fibonacci number generator.
		for i in range(goal.request.number + 1):
			result.sequence.append(fibonacci(i))

		# Let the action server know that we've succeeded in the action.
		goal.succeed()
		self.get_logger().info('Result: {0}'.format(result.sequence))

		# Return the result to the action server.
		return result


# This is the entry point for the node when called with ros2 run.
def main(args=None):
	# Initialize rclpy.
	rclpy.init(args=args)

	# Set up a node to do the work.
	server = FibonacciActionServer()

	# Give control over to ROS2
	rclpy.spin(server)


# This is the entry point for running the node directly from the command line.
if __name__ == '__main__':
	main()
