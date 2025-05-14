import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction
)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    # Create parameters dictionary
    parameters = {"use_sim_time": True}
    parameters["use_moveit"] = True

    urdf = os.path.join(
        get_package_share_directory("ariac_description"),
        "urdf/ariac_robots/ariac_robots.urdf.xacro",
    )

    # RViz config path
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("group2_ariac"), "config", "rviz_config.rviz"]
    )

    moveit_config = (
        MoveItConfigsBuilder("ariac_robots", package_name="ariac_moveit_config")
        .robot_description(urdf)
        .robot_description_semantic(file_path="config/ariac_robots.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .joint_limits(file_path="config/joint_limits.yaml")
        .moveit_cpp(
            file_path=get_package_share_directory("group2_ariac")
            + "/config/moveit_config.yaml"
        )
        .to_moveit_configs()
    )

    start_rviz = LaunchConfiguration("rviz")

    parameters.update(moveit_config.to_dict())

    # Python node that will start directly if RViz is not enabled
    robot_controller_node_py_direct = Node(
        package="group2_ariac",
        executable="robot_controller_node.py",
        output="screen",
        parameters=[parameters],
        condition=UnlessCondition(start_rviz),
    )
    
    # Python node that will be started by event handler if RViz is enabled
    robot_controller_node_py_after_rviz = Node(
        package="group2_ariac",
        executable="robot_controller_node.py",
        output="screen",
        parameters=[parameters],
    )

    # RViz Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
        condition=IfCondition(start_rviz),
    )

    # Register event handler to start Python node after RViz starts with a 5-second delay
    start_python_after_rviz = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=rviz_node,
            on_start=[
                # Add a TimerAction to delay the start of the Python node
                TimerAction(
                    period=5.0,  # 5-second delay
                    actions=[robot_controller_node_py_after_rviz]
                )
            ],
        ),
        condition=IfCondition(start_rviz),
    )

    # Move Group node
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("ariac_moveit_config"),
                "/launch",
                "/ariac_robots_moveit.launch.py",
            ]
        ),
        condition=IfCondition(str(parameters["use_moveit"])),
    )

    start_competition_node = Node(
        package="group2_ariac", 
        executable="start_competition_node",
        name="start_competition_node"
    )

    retrieve_orders_node = Node(
        package="group2_ariac", 
        executable="retrieve_orders",
        name="retrieve_orders"
    )

    tray_detector = Node(
        package="group2_ariac", 
        executable="tray_detector",
        name="tray_detector"
    )

    bin_parts_detector = Node(
        package="group2_ariac", 
        executable="bin_parts_detector",
        name="bin_parts_detector",
        # prefix=["gdbserver localhost:3000"],
    )

    ship_orders = Node(
        package="group2_ariac", 
        executable="ship_orders",
        name="ship_orders"
    )

    complete_orders = Node(
        package="group2_ariac", 
        executable="complete_orders",
        name="complete_orders"
    )

    end_competition_node = Node(
        package="group2_ariac", 
        executable="end_competition_node",
        name="end_competition_node"
    )

    conveyor_tracker_node = Node(
        package="group2_ariac", 
        executable="conveyor_tracker_node",
        name="conveyor_tracker_node",
        output="screen"
    )
    
    server_node = Node(
        package="group2_ariac", 
        executable="server",
        name="server",
        output="screen"
    )    

    # List of nodes to start
    nodes_to_start = [
        robot_controller_node_py_direct,  # Will only run if RViz is disabled
        move_group,
        rviz_node,
        # start_python_after_rviz,  # Will start Python node after RViz if RViz is enabled
        start_competition_node,
        retrieve_orders_node,
        tray_detector,
        bin_parts_detector,
        # ship_orders,
        # complete_orders,
        # end_competition_node,
        # conveyor_tracker_node,
        # server_node,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Launch RViz visualization?",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )