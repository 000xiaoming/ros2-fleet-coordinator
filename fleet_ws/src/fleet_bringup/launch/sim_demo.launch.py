from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def robot_description(robot_name: str) -> str:
    urdf_file = Path(get_package_share_directory("fleet_bringup")) / "urdf" / "simple_diffbot.urdf"
    prefix = f"{robot_name}_"
    description = urdf_file.read_text()
    return (
        description.replace("__ROBOT_NAME__", robot_name)
        .replace("__PREFIX__", prefix)
        .replace("__ODOM_FRAME__", f"{robot_name}_odom")
        .replace("__BASE_FRAME__", f"{robot_name}_base_link")
    )


def robot_nodes(robot_name: str, x: float, y: float, start_waypoint: str):
    description = robot_description(robot_name)
    use_sim_time = LaunchConfiguration("use_sim_time")

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=robot_name,
            parameters=[{"robot_description": description, "use_sim_time": use_sim_time}],
            output="screen",
        ),
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity",
                robot_name,
                "-topic",
                f"/{robot_name}/robot_description",
                "-x",
                str(x),
                "-y",
                str(y),
                "-z",
                "0.05",
            ],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "world", f"{robot_name}_odom"],
            output="screen",
        ),
        Node(
            package="robot_agent",
            executable="robot_agent_node",
            name=f"{robot_name}_agent",
            parameters=[
                str(Path(get_package_share_directory("fleet_bringup")) / "config" / "demo_map.yaml"),
                {
                    "robot_id": robot_name,
                    "x": x,
                    "y": y,
                    "start_waypoint": start_waypoint,
                    "execution_mode": "external_navigation",
                    "map_frame": "world",
                    "pose_topic": f"/{robot_name}/amcl_pose",
                    "goal_topic": f"/{robot_name}/fleet_goal_pose",
                    "navigation_result_topic": f"/{robot_name}/fleet_navigation_result",
                    "use_sim_time": use_sim_time,
                },
            ],
            output="screen",
        ),
        Node(
            package="fleet_runtime",
            executable="goal_follower_node",
            name=f"{robot_name}_goal_follower",
            parameters=[
                {
                    "robot_id": robot_name,
                    "goal_topic": f"/{robot_name}/fleet_goal_pose",
                    "cmd_vel_topic": f"/{robot_name}/cmd_vel",
                    "odom_topic": f"/{robot_name}/odom",
                    "pose_topic": f"/{robot_name}/amcl_pose",
                    "navigation_result_topic": f"/{robot_name}/fleet_navigation_result",
                    "use_sim_time": use_sim_time,
                }
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    bringup_share = Path(get_package_share_directory("fleet_bringup"))
    config_file = bringup_share / "config" / "demo_map.yaml"
    rviz_config = bringup_share / "rviz" / "fleet_simulation.rviz"
    world = bringup_share / "worlds" / "fleet_demo.world"
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    gui = LaunchConfiguration("gui")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory("gazebo_ros")) / "launch" / "gazebo.launch.py")
        ),
        launch_arguments={"world": str(world), "gui": gui, "verbose": "false"}.items(),
    )

    nodes = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true"),
        gazebo,
        Node(
            package="fleet_manager",
            executable="fleet_manager_node",
            parameters=[str(config_file), {"use_sim_time": use_sim_time}],
            output="screen",
        ),
        Node(
            package="path_planner",
            executable="path_planner_node",
            parameters=[str(config_file), {"use_sim_time": use_sim_time}],
            output="screen",
        ),
        Node(
            package="fleet_runtime",
            executable="fleet_visualization_node",
            parameters=[
                str(config_file),
                {
                    "map_frame": "world",
                    "use_sim_time": use_sim_time,
                },
            ],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(rviz_config)],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(use_rviz),
            output="screen",
        ),
    ]

    nodes.extend(robot_nodes("robot_1", 0.0, 0.0, "dock_a"))
    nodes.extend(robot_nodes("robot_2", 3.0, 1.5, "dock_c"))
    return LaunchDescription(nodes)
