#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import Int64


class ExampleQoS(Node):
	def __init__(self):
		super().__init__('qos_example')

		self.counter = 0

		# Latching topics should have depth 1, and durability of transient.
		latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

		# When creating the publisher, don't give a queue size, since this taken care of in the QoS settings.
		self.pub = self.create_publisher(Int64, 'counter', qos_profile=latching_qos)
		self.timer = self.create_timer(10, self.callback)

	def callback(self):
		number = Int64()
		number.data = self.counter

		self.pub.publish(number)
		self.get_logger().info(f'Published {number.data}')

		self.counter += 1


def main(args=None):
	rclpy.init(args=args)

	qos = ExampleQoS()

	rclpy.spin(qos)

	rclpy.shutdown()


if __name__ == '__main__':
	main()
