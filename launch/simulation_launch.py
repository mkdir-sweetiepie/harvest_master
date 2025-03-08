from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Gazebo 시뮬레이션 환경
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py",
                )
            ]
        ),
        launch_arguments={
            "world": os.path.join(
                get_package_share_directory("harvest_master"),
                "worlds",
                "crop_field.world",
            ),
            "verbose": "true",
        }.items(),
    )

    # 로봇 URDF 스폰
    robot_spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "harvest_robot",
            "-file",
            os.path.join(
                get_package_share_directory("robot_description"),
                "urdf",
                "harvest_robot.urdf",
            ),
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.1",
        ],
        output="screen",
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        arguments=[
            os.path.join(
                get_package_share_directory("robot_description"),
                "urdf",
                "harvest_robot.urdf",
            )
        ],
    )

    # 메인 시스템 launch
    main_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("harvest_master"),
                    "launch",
                    "harvest_master_launch.py",
                )
            ]
        ),
        launch_arguments={"use_sim_time": "true", "log_level": "info"}.items(),
    )

    return LaunchDescription(
        [
            gazebo_launch,
            robot_spawn,
            joint_state_publisher,
            robot_state_publisher,
            main_system_launch,
        ]
    )
