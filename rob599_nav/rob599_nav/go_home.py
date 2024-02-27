#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped


class Navigate(Node):
	def __init__(self):
		super().__init__('navigator')

		# This gives us access to the navigation API.
		self.navigator = BasicNavigator()

	def go_to(self, x, y):
		# Set a goal pose.  This is the starting location from our simulation, with an unspecified
		# orientation.
		goal_pose = PoseStamped()
		goal_pose.header.frame_id = 'map'
		goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
		goal_pose.pose.position.x = x
		goal_pose.pose.position.y = y
		goal_pose.pose.orientation.w = 1.0

		# Just go to that pose.  This call does not block.  Sending this preempts other tasks.
		self.navigator.goToPose(goal_pose)

		# Loop until we're done
		while not self.navigator.isTaskComplete():
			# Retrieve feedback on how we're doing.
			feedback = self.navigator.getFeedback()

			self.get_logger().info(f'Estimated time of arrival: {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9:.2f} seconds')

			# We can cancel tasks.
			if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
				self.navigator.cancelTask()

		# Did we succeed?
		match self.navigator.getResult():
			case TaskResult.SUCCEEDED:
				self.get_logger().info('Goal succeeded!')
			case TaskResult.CANCELED:
				self.get_logger().info('Goal was canceled!')
			case TaskResult.FAILED:
				self.get_logger().info('Goal failed!')
			case _:
				self.get_logger().info('Goal has an invalid return status!')


def main(args=None):
	# Initialize ROS
	rclpy.init(args=args)

	navigator = Navigate()
	navigator.go_to(1.0, 1.0)

	# Close things down politelty.
	rclpy.shutdown()


if __name__ == '__main__':
	main()
