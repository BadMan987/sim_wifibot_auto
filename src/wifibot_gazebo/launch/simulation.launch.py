import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    description_pkg = get_package_share_directory('wifibot_description')
    gazebo_pkg = get_package_share_directory('wifibot_gazebo')

    xacro_file = os.path.join(
        description_pkg,
        'urdf',
        'wifibot.urdf.xacro'
    )

    world_file = os.path.join(
        gazebo_pkg,
        'worlds',
        'indoor.world'
    )

    robot_description = ParameterValue(
        Command([
            'xacro ',
            xacro_file
        ]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True
            }
        ]
    )

    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            world_file,
            '-s',
            'libgazebo_ros_init.so',
            '-s',
            'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_wifibot',
        output='screen',
        arguments=[
            '-topic',
            'robot_description',
            '-entity',
            'wifibot',
            '-x',
            '0.0',
            '-y',
            '0.0',
            '-z',
            '0.2'
        ]
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_robot
    ])
