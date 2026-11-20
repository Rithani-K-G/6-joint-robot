import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    description_share = get_package_share_directory(
        'six_joint_description'
    )

    bringup_share = get_package_share_directory(
        'six_joint_bringup'
    )

    xacro_file = os.path.join(
        description_share,
        'urdf',
        'six_joint_robot.urdf.xacro'
    )

    controllers_file = os.path.join(
        bringup_share,
        'config',
        'controllers.yaml'
    )

    # Process Xacro
    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            xacro_file,
            ' controllers_file:=',
            controllers_file
        ]),
        value_type=str
    )

    # Start Gazebo Harmonic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': '-r empty.sdf'
        }.items()
    )

    # Robot State Publisher
    rsp = Node(
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

    # Spawn robot into Gazebo
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'six_joint_robot'
        ],
        output='screen'
    )

    # Joint State Broadcaster
    jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '30'
        ],
        output='screen'
    )

    # Arm Controller
    arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '30'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        rsp,

        TimerAction(
            period=3.0,
            actions=[spawn]
        ),

        TimerAction(
            period=5.0,
            actions=[jsb]
        ),

        TimerAction(
            period=7.0,
            actions=[arm]
        ),
    ])
