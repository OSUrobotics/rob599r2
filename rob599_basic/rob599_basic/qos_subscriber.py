#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import Int64


class LatchedSubscriber(Node):
	def __init__(self):
		super().__init__('latched_subscriber')

		# Latching topics should have depth 1, and durability of transient.
		latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

		self.sub = self.create_subscription(Int64, 'counter', self.callback, qos_profile=latching_qos)

	def callback(self, msg):
		self.get_logger().info(f'Got {msg.data}')


def main(args=None):
	rclpy.init(args=args)

	sub = LatchedSubscriber()

	rclpy.spin(sub)

	rclpy.shutdown()


if __name__ == '__main__':
	main()
