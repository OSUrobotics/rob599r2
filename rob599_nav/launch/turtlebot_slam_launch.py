from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
	return LaunchDescription([

		# Start the Turtlebot 3 simulation in Gazebo.
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource([
				os.path.join(
					get_package_share_directory('turtlebot3_gazebo'),
					'launch/turtlebot3_house.launch.py'
				)
			]),
			launch_arguments={
				'x_pose': '-1.5',
				'y_pose': '1.5'
			}.items()
		),

		# Start up the SLAM system.
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource([
				os.path.join(
					get_package_share_directory('turtlebot3_cartographer'),
					'launch/cartographer.launch.py'
				)
			]),
			launch_arguments={
				'use_sim_time': 'True'
			}.items()
		),

	])