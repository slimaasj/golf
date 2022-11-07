from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    pyIMUpub = Node(
        package="golf",
        executable="IMU",
    )
    pyMAGpub = Node(
        package="golf",
        executable="MAG"
    )

    ld.add_action(pyIMUpub)
    ld.add_action(pyMAGpub)

    return ld