from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        '/ros2_ws/',
        'picam_ros_params.yaml'
        )

    return LaunchDescription([

        Node(
            package='picam_ros2',
            executable='picam',
            name='picam_ros2',
            output='screen',
            emulate_tty=True,
            parameters=[config]
        )
    ])
