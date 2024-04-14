from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='adf_package',
            namespace='adf_package',
            executable='motion_sensor_node',
            name='motion_sensor_node'
        ),
        Node(
            package='adf_package',
            namespace='adf_package',
            executable='bowl_weight_node',
            name='bowl_weight_node'
        ),
        Node(
            package='adf_package',
            namespace='adf_package',
            executable='actuation_node',
            name='actuation_node'
        ),
        Node(
            package='adf_package',
            namespace='adf_package',
            executable='video_recorder_node',
            name='video_recorder_node'
        )
    ])
