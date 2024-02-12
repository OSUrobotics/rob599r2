import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # Use the original publisher node, but rename it to 'generator'.
        launch_ros.actions.Node(
            package='rob599_basic',
            executable='publisher',
            name='generator',
            remappings=[
                ('counter', 'number'),
            ]
        ),

        # The doubler node.
        launch_ros.actions.Node(
            package='rob599_basic',
            executable='doubler',
            name='doubler',
            remappings=[
                ('multiplied', 'doubled'),
            ]
            ),

        ])
