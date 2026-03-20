# Project Status

Date: 2026-03-20

## Goal

Create a standalone ROS 2 C++ project for a multi-robot fleet coordinator inside `/home/bruce/project-a` without affecting other projects.

## Documentation Coverage

The project documentation is split by purpose:

- `README.md`: overview and quick start
- `STATUS.md`: current behavior, verification status, limitations, and next steps
- `BUGS.md`: all meaningful bugs and environmental blockers
- `PROGRESS.md`: chronological record of meaningful project changes
- `TECHNICAL_APPROACH.md`: technical design, chosen technologies, rationale, and comparisons with alternatives

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
- `BUGS.md`
- `TECHNICAL_APPROACH.md`
- `fleet_ws/src/fleet_msgs/msg/RobotState.msg`
- `fleet_ws/src/fleet_msgs/msg/Task.msg`
- `fleet_ws/src/fleet_msgs/srv/SubmitTask.srv`
- `fleet_ws/src/fleet_msgs/action/ExecuteTask.action`
- `fleet_ws/src/fleet_manager/src/fleet_manager_node.cpp`
- `fleet_ws/src/robot_agent/src/robot_agent_node.cpp`
- `fleet_ws/src/path_planner/src/path_planner_node.cpp`
- `fleet_ws/src/fleet_bringup/launch/demo.launch.py`
- `fleet_ws/src/fleet_bringup/config/demo_map.yaml`
- `fleet_ws/src/fleet_bringup/scripts/submit_demo_task.sh`

## Current Behavior

### `robot_agent`

- Publishes `fleet_msgs/msg/RobotState` on `robot_states`
- Parameters:
  - `robot_id`
  - `x`
  - `y`
  - `start_waypoint`
  - `waypoint_positions`
- Subscribes to `task_assignments`
- Publishes `idle`, `executing`, and `completed` state transitions
- Simulates waypoint-by-waypoint progress for an assigned route
- Resolves waypoint coordinates from parameters, with a built-in fallback map

### `fleet_manager`

- Subscribes to `robot_states`
- Stores latest state per robot
- Provides `submit_task` service using `fleet_msgs/srv/SubmitTask`
- Queues pending tasks
- Requests a route from `path_planner` before assignment
- Publishes `fleet_msgs/msg/TaskAssignment` to the selected robot
- Preserves assignment state until the agent reports execution or completion

### `path_planner`

- Provides a `plan_route` service using a parameter-driven waypoint graph
- Returns BFS-planned routes from robot start waypoint to pickup and then dropoff
- Loads demo graph topology from `fleet_bringup/config/demo_map.yaml`

### `fleet_bringup`

- Launches:
  - `fleet_manager`
  - `path_planner`
  - `robot_agent_1`
  - `robot_agent_2`
- Ships:
  - `config/demo_map.yaml` for the demo graph and waypoint coordinates
  - `submit_demo_task.sh` for repeatable task submission

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

### 3. Executable route-assignment flow

Problem:
The previous project state only queued and internally assigned tasks. No planner output was requested, no assignment was sent to agents, and agents had no way to simulate task execution.

Fix:

- Added `fleet_msgs/msg/TaskAssignment`
- Added `fleet_msgs/srv/PlanRoute`
- Extended `fleet_msgs/msg/RobotState` with `current_waypoint`
- Implemented a graph-backed `plan_route` service in `path_planner`
- Updated `fleet_manager` to request routes asynchronously and publish assignments
- Updated `robot_agent` to accept assignments and publish execution/completion progress

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

Current verification command used:

```bash
cd /home/bruce/project-a/ros2-fleet-coordinator/fleet_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fleet_msgs fleet_manager robot_agent path_planner fleet_bringup
```

Additional 2026-03-20 verification:

```bash
cd /home/bruce/project-a/ros2-fleet-coordinator
bash -n fleet_ws/src/fleet_bringup/scripts/submit_demo_task.sh

cd /home/bruce/project-a/ros2-fleet-coordinator/fleet_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_agent path_planner fleet_bringup
colcon build --packages-select fleet_bringup
```

Launch startup was also tested on 2026-03-19 with:

```bash
cd /home/bruce/project-a/ros2-fleet-coordinator/fleet_ws
export ROS_LOG_DIR=/home/bruce/project-a/ros2-fleet-coordinator/fleet_ws/log/ros
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fleet_bringup demo.launch.py
```

Result:

- Processes start
- `fleet_manager` receives and tracks robot state updates
- `path_planner` starts with the configured waypoint graph
- Agent nodes publish waypoint-aware idle state logs
- DDS transport still reports socket permission errors in this sandbox environment

Task submission with `ros2 service call` could not be verified inside the sandbox because a separate CLI participant remained blocked waiting for service discovery under the same DDS socket restrictions.

This means:

- build and packaging are good
- in-process node startup and basic topic traffic are working
- sandbox restrictions still prevent full CLI-driven service verification
- full end-to-end assignment execution should be tested in a normal host shell

## Current Limitations

- Demo map data is still ROS parameter YAML rather than a richer map format
- No conflict detection or reservation logic
- `fleet_manager` assignment is still simple first-idle selection
- Robot battery is static
- No RViz visualization yet
- Full CLI-driven runtime verification remains blocked in this sandbox by DDS socket restrictions

## Technical Approach Summary

The adopted implementation approach is intentionally conservative for V1:

- ROS 2 is used as the robotics-native communication and packaging layer
- C++ is used for the core nodes to stay close to common production robotics runtime patterns
- `fleet_msgs` centralizes domain-specific interfaces
- `fleet_manager`, `path_planner`, and `robot_agent` are separated by responsibility
- Route planning is implemented as a service because `fleet_manager` requires a request/response planner dependency before dispatch
- Robot state and assignment lifecycle are represented through topic traffic
- BFS on a waypoint graph was chosen as the smallest correct planning strategy for the current unweighted demo map
- Demo graph topology and waypoint coordinates are now loaded through launch-time parameters

Detailed rationale and comparisons with alternatives are documented in `TECHNICAL_APPROACH.md`.

## Recommended Next Steps

### Next coding step

1. Verify full assignment and completion flow outside the sandbox with `submit_demo_task.sh`
2. Add conflict reservation and smarter scheduling
3. Add visualization or richer runtime introspection

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

- `fleet_ws/src/fleet_bringup/scripts/submit_demo_task.sh`
- `fleet_ws/src/path_planner/src/path_planner_node.cpp`
- `fleet_ws/src/fleet_bringup/launch/demo.launch.py`
- `fleet_ws/src/fleet_bringup/config/demo_map.yaml`

## Suggested Design Direction

Keep V1 graph-based and simulated.

Do not jump to Gazebo or Nav2 yet.

A strong incremental path is:

1. configurable map loading
2. repeatable end-to-end demo task submission
3. conflict reservation
4. priority-aware scheduling
5. visualization

That keeps the project manageable while still looking like real ROS 2 systems work.
