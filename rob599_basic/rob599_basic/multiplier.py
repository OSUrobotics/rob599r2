#!/usr/bin/env python3


# Example of a transformer node in ROS 2
#
# multiplier.py
#
# Bill Smart
#
# This node subscribes to a topic with Int64s.  It multiplies the numbers by a constant,
# and then republishes them on another topic.


# Every Python node in ROS2 should include these lines.  rclpy is the basic Python
# ROS2 stuff, and Node is the class we're going to use to set up the node.
import rclpy
from rclpy.node import Node

# We're going to publish an Int64.  This is the same as ROS.
from std_msgs.msg import Int64

# Random gaussian noise
from random import gauss


class ValueManipulator(Node):
	'''
	This node subscribes to a topic publishing Int64s, runs a function on each number if receives,
	and repubishes the result on an outbound topic.
	'''
	def __init__(self, f):
		# Initialize the parent class, giving it a name.  The idiom is to use the
		# super() class.
		super().__init__('multiplier')

		# Store the manipulation function.
		self.f = f

		# Create the publisher first, to make sure it's available before we start the subscriber.
		self.pub = self.create_publisher(Int64, 'multiplied', 10)

		# Create the subscriber.
		self.sub = self.create_subscription(Int64, 'number', self.callback, 10)


	# This callback will be called whenever we receive a new message on the topic.
	def callback(self, msg):
		# Multiply the data element of the message.
		msg.data = self.f(msg.data)

		# Republish the message.  Reusing the original message is slightly more efficient
		# than creating a new one.
		self.pub.publish(msg)


# We're going to create a number of entry points for the node here.  Start with doubling
# the value.
def doubler(args=None):
	run_node(lambda x: x * 2, args)


# Add random gaussian noise
def noiser(args=None):
	run_node(lambda x: int(gauss(x, 1)), args)


# This is a general-purpose entry point.	
def run_node(f, args=None):
	# Initialize rclpy.  We should do this every time.
	rclpy.init(args=args)

	# Make a node class.  The idiom in ROS2 is to encapsulte everything in a class
	# that derives from Node.
	manip = ValueManipulator(f)

	# The spin() call gives control over to ROS2, and it now takes a Node-derived
	# class as a parameter.
	rclpy.spin(manip)

	# Make sure we shutdown everything cleanly.  This should happen, even if we don't
	# include this line, but you should do it anyway.
	rclpy.shutdown()


# If we run the node as a script, then we're going to start here.
if __name__ == '__main__':
	# The idiom in ROS2 is to set up a main() function and to call it from the entry
	# point of the script.
	doubler()