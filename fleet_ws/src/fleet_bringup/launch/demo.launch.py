from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = Path(get_package_share_directory("fleet_bringup")) / "config" / "demo_map.yaml"
    execution_mode = LaunchConfiguration("execution_mode")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "execution_mode",
                default_value="simulated",
                description="robot_agent execution mode: simulated or external_navigation",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="propagate simulated time into fleet nodes",
            ),
            Node(
                package="fleet_manager",
                executable="fleet_manager_node",
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="path_planner",
                executable="path_planner_node",
                parameters=[str(config_file), {"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="robot_agent",
                executable="robot_agent_node",
                name="robot_agent_1",
                parameters=[
                    str(config_file),
                    {
                        "robot_id": "robot_1",
                        "x": 0.0,
                        "y": 0.0,
                        "start_waypoint": "dock_a",
                        "execution_mode": execution_mode,
                        "pose_topic": "/robot_1/amcl_pose",
                        "goal_topic": "/robot_1/fleet_goal_pose",
                        "navigation_result_topic": "/robot_1/fleet_navigation_result",
                        "use_sim_time": use_sim_time,
                    }
                ],
                output="screen",
            ),
            Node(
                package="robot_agent",
                executable="robot_agent_node",
                name="robot_agent_2",
                parameters=[
                    str(config_file),
                    {
                        "robot_id": "robot_2",
                        "x": 2.0,
                        "y": 1.0,
                        "start_waypoint": "dock_c",
                        "execution_mode": execution_mode,
                        "pose_topic": "/robot_2/amcl_pose",
                        "goal_topic": "/robot_2/fleet_goal_pose",
                        "navigation_result_topic": "/robot_2/fleet_navigation_result",
                        "use_sim_time": use_sim_time,
                    }
                ],
                output="screen",
            ),
        ]
    )
