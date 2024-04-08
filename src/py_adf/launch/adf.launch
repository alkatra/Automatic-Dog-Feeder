from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='automatic_dog_feeder',
            executable='motion_sensor_node.py',
            name='motion_sensor_node',
            output='screen',
        ),
        Node(
            package='automatic_dog_feeder',
            executable='video_recorder_node.py',
            name='video_recorder_node',
            output='screen',
        ),
        Node(
            package='automatic_dog_feeder',
            executable='bowl_weight_publisher.py',
            name='bowl_weight_publisher',
            output='screen',
        ),
        Node(
            package='automatic_dog_feeder',
            executable='actuation_node.py',
            name='actuation_node',
            output='screen',
        ),
    ])