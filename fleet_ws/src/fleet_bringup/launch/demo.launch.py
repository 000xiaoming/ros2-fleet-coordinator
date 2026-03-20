from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = Path(get_package_share_directory("fleet_bringup")) / "config" / "demo_map.yaml"

    return LaunchDescription(
        [
            Node(
                package="fleet_manager",
                executable="fleet_manager_node",
                output="screen",
            ),
            Node(
                package="path_planner",
                executable="path_planner_node",
                parameters=[str(config_file)],
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
                    }
                ],
                output="screen",
            ),
        ]
    )
