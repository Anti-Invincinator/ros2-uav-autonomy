from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensors',
            executable='gps_publisher',
            output='screen'
        ),
        Node(
            package='sensors',
            executable='imu_publisher',
            output='screen'
        )
    ])
