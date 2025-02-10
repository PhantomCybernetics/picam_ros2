from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        '/ros2_ws/',
        'picam_ros2_params.yaml'
        )

    return LaunchDescription([

        Node(
            package='picam_ros2',
            executable='picam',
            output='screen',
            emulate_tty=True,
            parameters=[config]
        )
    ])
