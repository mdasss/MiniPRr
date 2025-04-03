#Modified launch file from https://github.com/ros/urdf_launch/blob/main/launch/display.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    ld = LaunchDescription()

    urdf_launch_package = FindPackageShare('urdf_launch')


    ld.add_action(DeclareLaunchArgument(name='jsp_gui', default_value='true', choices=['true', 'false'],
                                        description='Flag to enable joint_state_publisher_gui'))

    default_rviz_config_path = PathJoinSubstitution([urdf_launch_package, 'config', 'urdf.rviz'])

    ld.add_action(DeclareLaunchArgument(name='rviz_config', default_value=default_rviz_config_path,
                                        description='Absolute path to rviz config file'))

    # need to manually pass configuration in because of https://github.com/ros2/launch/issues/313

    ld.add_action(DeclareLaunchArgument(
        name='urdf_package',
        default_value='qube_bringup',
        description='Name of the package containing the URDF'
    ))

    ld.add_action(DeclareLaunchArgument(
        name='urdf_package_path',
        default_value='urdf/controlled_qube.urdf.xacro',
        description='Path within the package to the URDF file'
    ))

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([urdf_launch_package, 'launch', 'description.launch.py']),
        launch_arguments={
            'urdf_package': LaunchConfiguration('urdf_package'),
            'urdf_package_path': LaunchConfiguration('urdf_package_path')}.items()
    ))



    ld.add_action(Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('jsp_gui'))
    ))

    ld.add_action(Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('jsp_gui'))
    ))





    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    ))
    # Include qube_driver.launch.py
    qube_driver_launch_path = PathJoinSubstitution([
        FindPackageShare("qube_driver"),
        "launch",
        "qube_driver.launch.py"
    ])

    ld.add_action(IncludeLaunchDescription(qube_driver_launch_path))

    return ld