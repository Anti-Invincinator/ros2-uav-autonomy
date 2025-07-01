from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'frequency': 30.0,
                'sensor_timeout': 0.1,
                'two_d_mode': False,
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_link_frame': 'base_link',
                'world_frame': 'odom',
                'odom0': '/gps/fix',
                'odom0_config': [True, True, False,
                                 False, False, False,
                                 False, False, False,
                                 False, False, False,
                                 False, False, False],
                'imu0': '/imu/data',
                'imu0_config': [False, False, False,
                                True, True, True,
                                False, False, False,
                                True, True, True,
                                False, False, False],
                'imu0_differential': False,
                'imu0_queue_size': 5,
                'imu0_remove_gravitational_acceleration': True
            }]
        )
    ])
