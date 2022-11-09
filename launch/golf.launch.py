import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='golf',
            namespace='IMU',
            executable='pyIMUpub',
            name='pyIMUpub'
        ),
        Node(
            package='golf',
            namespace='MAG',
            executable='pyMAGpub',
            name='pyMAGpub'
        ),
        Node(
            package='golf',
            namespace='GPS',
            executable='pyGPSpub',
            name='pyGPSpub'
        ),
        Node(
            package='golf',
            namespace='State_Publisher',
            executable='state_publisher',
            name='state_publisher',
        ),
    ])