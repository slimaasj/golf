import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
import launch_ros

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='golf').find('golf')
    default_model_path = os.path.join(pkg_share, 'golfball_robot.urdf.xml')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    robot_localization_node_map = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[os.path.join(pkg_share, 'ekf_map.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    robot_localization_node_odom = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[os.path.join(pkg_share, 'ekf_odom.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )


    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        robot_state_publisher_node,
        robot_localization_node_map,
        robot_localization_node_odom,

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
            namespace='Motor',
            executable='pyMotor',
            name='pyMotor'
        )
    ])