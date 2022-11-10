import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
import launch_ros

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='golf').find('golfball_robot')
    default_model_path = os.path.join(pkg_share, 'src/models/golfball_robot.urdf')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    robot_localization_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )


    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        robot_state_publisher_node,
        robot_localization_node,

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
    ])