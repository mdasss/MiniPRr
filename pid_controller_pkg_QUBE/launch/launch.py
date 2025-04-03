from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import xacro
from launch.substitutions import Command


def generate_launch_description():
    # Load URDF from xacro
    params = {
    'device': LaunchConfiguration('device'),
    'baud_rate': LaunchConfiguration('baud_rate'),
    'simulation': LaunchConfiguration('simulation')
}
    
    xacro_path = os.path.join(
        get_package_share_directory("pid_controller_pkg_QUBE"),
        "urdf",
        "controlled_qube.urdf.xacro"
    )
    robot_description = {
    'robot_description': Command([
        'xacro ', PathJoinSubstitution([
            FindPackageShare("pid_controller_pkg_QUBE"),
            'urdf',
            'controlled_qube.urdf.xacro'
        ]),
        ' device:=', LaunchConfiguration('device'),
        ' baud_rate:=', LaunchConfiguration('baud_rate'),
        ' simulation:=', LaunchConfiguration('simulation')
    ])
}
    # Load RViz config
    rviz_config_file = os.path.join(
        get_package_share_directory("pid_controller_pkg_QUBE"),
        "config",
        "configuration_rviz.rviz"
    )

    # Load controller config
    controller_config = os.path.join(
        get_package_share_directory("pid_controller_pkg_QUBE"),
        "config",
        "controllers.yaml"
    )

    # Include GUI launch
    #gui_launch = IncludeLaunchDescription(
      #  PythonLaunchDescriptionSource(
       #     PathJoinSubstitution([
        #        FindPackageShare('qube_driver'),
         #       'launch',
         #       'qube_driver.launch.py'
         #   ])
       # )
    #)

    return LaunchDescription([
        # PID arguments
        DeclareLaunchArgument('kp', default_value='1.0'),
        DeclareLaunchArgument('ki', default_value='0.0'),
        DeclareLaunchArgument('kd', default_value='0.0'),
        DeclareLaunchArgument('device', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud_rate', default_value='115200'),
        DeclareLaunchArgument('simulation', default_value='false'),


        # PID Node
        Node(
            package='pid_controller_pkg_QUBE',
            executable='pid_controller_node_QUBE',
            name='pid_controller',
            output='screen',
            parameters=[{
                'p': LaunchConfiguration('kp'),
                'i': LaunchConfiguration('ki'),
                'd': LaunchConfiguration('kd')
            }]
        ),

        # Robot state publisher
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
            remappings=[('/robot_description', 'robot_description')]
),


        # ros2_control node
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='ros2_control_node',
            output='screen',
            parameters=[
                robot_description,
                controller_config
    ]
),

        # Spawn joint state broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        ),

        # Spawn velocity controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['velocity_controller'],
            output='screen'
        ),

        # RViz
        Node(
            package='rviz2',
            namespace='RViz',
            executable='rviz2',
            name='RViz',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),

        # GUI reference tool
        gui_launch
    ])

