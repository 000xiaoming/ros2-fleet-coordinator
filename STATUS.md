# Project Status

Date: 2026-03-18

## Goal

Create a standalone ROS 2 C++ project for a multi-robot fleet coordinator inside `/home/bruce/project-a` without affecting other projects.

## Project Directory

- Root: `/home/bruce/project-a/ros2-fleet-coordinator`
- Workspace: `/home/bruce/project-a/ros2-fleet-coordinator/fleet_ws`

## What Was Created

The project was initialized as an isolated ROS 2 workspace with these packages:

- `fleet_msgs`
- `fleet_manager`
- `robot_agent`
- `path_planner`
- `fleet_bringup`

Main files:

- `README.md`
- `fleet_ws/src/fleet_msgs/msg/RobotState.msg`
- `fleet_ws/src/fleet_msgs/msg/Task.msg`
- `fleet_ws/src/fleet_msgs/srv/SubmitTask.srv`
- `fleet_ws/src/fleet_msgs/action/ExecuteTask.action`
- `fleet_ws/src/fleet_manager/src/fleet_manager_node.cpp`
- `fleet_ws/src/robot_agent/src/robot_agent_node.cpp`
- `fleet_ws/src/path_planner/src/path_planner_node.cpp`
- `fleet_ws/src/fleet_bringup/launch/demo.launch.py`

## Current Behavior

### `robot_agent`

- Publishes `fleet_msgs/msg/RobotState` on `robot_states`
- Parameters:
  - `robot_id`
  - `x`
  - `y`
- Currently publishes an `idle` state once per second

### `fleet_manager`

- Subscribes to `robot_states`
- Stores latest state per robot
- Provides `submit_task` service using `fleet_msgs/srv/SubmitTask`
- Queues pending tasks
- Assigns the next pending task to the first idle robot
- Preserves assigned task state internally so repeated `idle` publications from agents do not immediately re-free the robot

### `path_planner`

- Only a scaffold node with heartbeat logging
- No actual route planning implemented yet

### `fleet_bringup`

- Launches:
  - `fleet_manager`
  - `path_planner`
  - `robot_agent_1`
  - `robot_agent_2`

## Important Fixes Already Made

### 1. Package type fix

Problem:
`colcon list` initially detected packages as `ros.catkin`, which prevented the workspace from overlaying correctly.

Fix:
Added this to every `package.xml`:

```xml
<export>
  <build_type>ament_cmake</build_type>
</export>
```

After that, `colcon list` reported all packages as `ros.ament_cmake`.

### 2. Launch logging path workaround

Problem:
`ros2 launch` tried to write logs under `~/.ros/log`, which is read-only in the sandbox.

Workaround used during verification:

```bash
export ROS_LOG_DIR=/home/bruce/project-a/ros2-fleet-coordinator/fleet_ws/log/ros
```

## Verified Working State

These packages build successfully:

- `fleet_msgs`
- `fleet_manager`
- `robot_agent`
- `path_planner`
- `fleet_bringup`

Build command used:

```bash
cd /home/bruce/project-a/ros2-fleet-coordinator/fleet_ws
source /opt/ros/humble/setup.bash
colcon build --cmake-clean-cache --packages-select fleet_msgs fleet_manager robot_agent path_planner fleet_bringup
```

Launch startup was also tested with:

```bash
cd /home/bruce/project-a/ros2-fleet-coordinator/fleet_ws
export ROS_LOG_DIR=/home/bruce/project-a/ros2-fleet-coordinator/fleet_ws/log/ros
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fleet_bringup demo.launch.py
```

Result:

- Processes start
- Agent nodes print state publication logs
- DDS transport reports socket permission errors in this sandbox environment

This means:

- build and packaging are good
- local sandbox prevents full ROS 2 transport verification
- full runtime behavior should be tested in a normal shell on the host machine

## Current Limitations

- No real path planning yet
- No route graph yet
- No task execution in `robot_agent`
- No task completion feedback
- No conflict detection or reservation logic
- `fleet_manager` assignment is simple first-idle selection
- Robot battery is static
- No RViz visualization yet

## Recommended Next Steps

### Next coding step

Implement actual task execution flow:

1. Add a planner interface that returns waypoint routes
2. Let `fleet_manager` request a route before assignment
3. Let `robot_agent` receive an assigned task and simulate waypoint progress
4. Publish task completion back to `fleet_manager`
5. Mark robot idle again after completion

### After that

- Add a simple graph map in config
- Add conflict checking on nodes/edges
- Add priority-aware scheduling
- Add visualization

## Fast Resume Notes For Tomorrow

If resuming tomorrow, start here:

```bash
cd /home/bruce/project-a/ros2-fleet-coordinator/fleet_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Useful checks:

```bash
colcon list
colcon build --packages-select fleet_msgs fleet_manager robot_agent path_planner fleet_bringup
```

Most likely next files to edit:

- `fleet_ws/src/fleet_manager/src/fleet_manager_node.cpp`
- `fleet_ws/src/robot_agent/src/robot_agent_node.cpp`
- `fleet_ws/src/path_planner/src/path_planner_node.cpp`
- possibly new interfaces in `fleet_ws/src/fleet_msgs/`

## Suggested Design Direction

Keep V1 graph-based and simulated.

Do not jump to Gazebo or Nav2 yet.

A strong incremental path is:

1. task queue
2. route generation
3. agent progress simulation
4. completion reporting
5. conflict reservation

That keeps the project manageable while still looking like real ROS 2 systems work.
