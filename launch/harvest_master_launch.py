from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation time if true"
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Log level for nodes (debug, info, warn, error)",
    )

    # 메인 관제 GUI 노드
    harvest_master_node = Node(
        package="harvest_master",
        executable="harvest_master",
        name="harvest_master_gui",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    # YOLOv12 비전 노드 (별도 패키지로 가정)
    vision_node = Node(
        package="crop_vision",
        executable="yolov12_detector",
        name="crop_vision_node",
        output="screen",
        parameters=[
            {
                "model_path": "/path/to/yolov12_crop_model.pt",
                "confidence_threshold": 0.7,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    # TSP 경로 계획 노드
    planning_node = Node(
        package="path_planning",
        executable="tsp_planner",
        name="harvest_planner_node",
        output="screen",
        parameters=[
            {
                "algorithm": "held_karp",
                "max_crops": 8,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    # RRT* 경로 생성 노드
    rrt_node = Node(
        package="path_planning",
        executable="rrt_star_planner",
        name="rrt_planner_node",
        output="screen",
        parameters=[
            {
                "max_iterations": 5000,
                "step_size": 0.1,
                "goal_tolerance": 0.05,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    # FoundationPose 6D 추정 노드
    foundation_pose_node = Node(
        package="foundation_pose",
        executable="pose_estimator",
        name="foundation_pose_node",
        output="screen",
        parameters=[
            {
                "mesh_path": "/path/to/crop_mesh.obj",
                "confidence_threshold": 0.8,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    # 로봇 제어 노드
    robot_control_node = Node(
        package="robot_control",
        executable="manipulator_controller",
        name="robot_controller_node",
        output="screen",
        parameters=[
            {
                "dof": 6,
                "joint_limits": [
                    [-180.0, 180.0],  # Joint 1
                    [-90.0, 90.0],  # Joint 2
                    [-180.0, 180.0],  # Joint 3
                    [-180.0, 180.0],  # Joint 4
                    [-180.0, 180.0],  # Joint 5
                    [-180.0, 180.0],  # Joint 6
                ],
                "max_velocity": [1.0, 1.0, 1.0, 2.0, 2.0, 2.0],
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    # 모터 제어 노드 (기존 제공된 코드 기반)
    motor_control_node = Node(
        package="motor_control",
        executable="motor_controller",
        name="motor_control_node",
        output="screen",
        parameters=[
            {
                "dc_motor_count": 4,
                "dynamixel_count": 2,
                "serial_port_stm": "/dev/ttyUSB1",
                "serial_port_u2d2": "/dev/ttyACM0",
                "baudrate": 115200,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            log_level_arg,
            # 메인 관제 시스템
            harvest_master_node,
            # 비전 시스템
            vision_node,
            foundation_pose_node,
            # 경로 계획 시스템
            planning_node,
            rrt_node,
            # 로봇 제어 시스템
            robot_control_node,
            motor_control_node,
        ]
    )
