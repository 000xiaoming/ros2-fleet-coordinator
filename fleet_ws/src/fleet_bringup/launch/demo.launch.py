from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
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
                output="screen",
            ),
            Node(
                package="robot_agent",
                executable="robot_agent_node",
                name="robot_agent_1",
                parameters=[{"robot_id": "robot_1", "x": 0.0, "y": 0.0}],
                output="screen",
            ),
            Node(
                package="robot_agent",
                executable="robot_agent_node",
                name="robot_agent_2",
                parameters=[{"robot_id": "robot_2", "x": 2.0, "y": 1.0}],
                output="screen",
            ),
        ]
    )
