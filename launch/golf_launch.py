from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='golf',
            namespace='IMU',
            executable='pyIMUpub',
            name='IMU'
        ),
        Node(
            package='golf',
            namespace='MAG',
            executable='pyMAGpub',
            name='MAG'
        )
    ])