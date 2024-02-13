import launch
import launch_ros.actions

NUMBER=13

def generate_launch_description():
	cascade = [launch_ros.actions.Node(
		package='rob599_basic',
		executable='publisher',
		name='generator',
		remappings=[
			('counter', f'in0'),
		])
	]

	# Add all but the last node.
	cascade.extend([launch_ros.actions.Node(
		package='rob599_basic',
		executable='doubler',
		name=f'doubler{i}',
		remappings=[
			('number', f'in{i}'),
			('multiplied', f'in{i + 1}'),
		]) for i in range(NUMBER - 1)])

	# The last node is special, since we don't remap the output.
	cascade.append(launch_ros.actions.Node(
		package='rob599_basic',
		executable='doubler',
		name=f'doubler{NUMBER - 1}',
		remappings=[
			('number', f'in{NUMBER - 1}'),
		])
	)

	return launch.LaunchDescription(cascade)

