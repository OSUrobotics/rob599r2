#!/usr/bin/env python3


# Example of parameters in a node.
#
# params.py
#
# Bill Smart
#
# This node shows how to use parameters in Python.


# Every Python node in ROS2 should include these lines.  rclpy is the basic Python
# ROS2 stuff, and Node is the class we're going to use to set up the node.
import rclpy
from rclpy.node import Node

# We're going to publish an Int64.  This is the same as ROS.
from std_msgs.msg import Int64


class ParamDemo(Node):
	'''
	ROS parameter demo node.
	'''
	def __init__(self):
		# Initialize the parent class, giving it a name.  The idiom is to use the
		# super() class.
		super().__init__('param_demo')

		# You have to declare a parameter before you use it.  The Python API has
		# no implementation of on_parameter_event(), which means you can't associate
		# a callback with parameter changes.
		self.declare_parameter('speed', 10.0)

		self.timer = self.create_timer(1, self.callback)

	# This callback will be called periodically.  Since on_callback_event() is not
	# implemented in Python, we're reduced to polling for the value.
	def callback(self):
		speed = self.get_parameter('speed').get_parameter_value().double_value
		self.get_logger().info(f'Parameter value is: {speed}')


# This is the entry point
def main(args=None):
	# Initialize rclpy.  We should do this every time.
	rclpy.init(args=args)

	# Make a node class.  The idiom in ROS2 is to encapsulte everything in a class
	# that derives from Node.
	demo = ParamDemo()

	# The spin() call gives control over to ROS2, and it now takes a Node-derived
	# class as a parameter.
	rclpy.spin(demo)

	# Make sure we shutdown everything cleanly.  This should happen, even if we don't
	# include this line, but you should do it anyway.
	rclpy.shutdown()


# If we run the node as a script, then we're going to start here.
if __name__ == '__main__':
	# The idiom in ROS2 is to set up a main() function and to call it from the entry
	# point of the script.
	main()
