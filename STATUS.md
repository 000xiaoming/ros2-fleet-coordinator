# Project Status

Date: 2026-03-21

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
- `fleet_ws/src/fleet_msgs/msg/TaskAssignment.msg`
- `fleet_ws/src/fleet_msgs/srv/SubmitTask.srv`
- `fleet_ws/src/fleet_msgs/srv/PlanRoute.srv`
- `fleet_ws/src/fleet_msgs/action/ExecuteTask.action`
- `fleet_ws/src/fleet_manager/src/fleet_manager_node.cpp`
- `fleet_ws/src/robot_agent/src/robot_agent_node.cpp`
- `fleet_ws/src/path_planner/src/path_planner_node.cpp`
- `fleet_ws/src/fleet_bringup/launch/demo.launch.py`
- `fleet_ws/src/fleet_bringup/config/demo_map.yaml`
- `fleet_ws/src/fleet_bringup/scripts/submit_demo_task.sh`
- `fleet_ws/src/fleet_bringup/scripts/check_planner_failure_path.sh`

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
- Drops planner-rejected front-of-queue tasks so one invalid task does not stall the queue
- Preserves assignment state until the agent reports execution or completion

### `path_planner`

- Provides a `plan_route` service using a parameter-driven waypoint graph
- Returns BFS-planned routes from robot start waypoint to pickup and then dropoff
- Rejects tasks whose start, pickup, or dropoff waypoints are not on the configured graph
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
  - `check_planner_failure_path.sh` for end-to-end planner-failure recovery verification

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
- Extended `RobotState` with `current_waypoint`
- Implemented a graph-backed `plan_route` service in `path_planner`
- Updated `fleet_manager` to request routes asynchronously and publish assignments
- Updated `robot_agent` to accept assignments and publish execution/completion progress

### 4. Queue-stall fix for planner rejection

Problem:
If the planner rejected the front task in `pending_tasks_`, `fleet_manager` logged the failure but left that task at the front of the queue, blocking later valid tasks.

Fix:

- Updated `fleet_manager` to remove planner-rejected front-of-queue tasks
- Added a runtime recovery check script that proves a valid task still runs after an invalid one is rejected

## Verified Working State

These packages build successfully:

- `fleet_msgs`
- `fleet_manager`
- `robot_agent`
- `path_planner`
- `fleet_bringup`

Build commands used:

```bash
cd /home/bruce/project-a/ros2-fleet-coordinator/fleet_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fleet_msgs fleet_manager robot_agent path_planner fleet_bringup
colcon build --event-handlers console_direct+ --packages-select fleet_manager
colcon build --packages-select fleet_bringup
```

Additional 2026-03-21 verification:

```bash
cd /home/bruce/project-a/ros2-fleet-coordinator
bash -n fleet_ws/src/fleet_bringup/scripts/submit_demo_task.sh
bash -n fleet_ws/src/fleet_bringup/scripts/check_planner_failure_path.sh

cd /home/bruce/project-a/ros2-fleet-coordinator/fleet_ws
export ROS_LOG_DIR=/home/bruce/project-a/ros2-fleet-coordinator/fleet_ws/log/ros
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fleet_bringup demo.launch.py
```

From another shell:

```bash
cd /home/bruce/project-a/ros2-fleet-coordinator
source /opt/ros/humble/setup.bash
source fleet_ws/install/setup.bash
ros2 run fleet_bringup submit_demo_task.sh --task-id demo_001

cd /home/bruce/project-a/ros2-fleet-coordinator/fleet_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run fleet_bringup check_planner_failure_path.sh
```

Result:

- `submit_demo_task.sh` is runnable through `ros2 run`
- `submit_task` accepts a demo task
- `fleet_manager` queues the task and assigns it to an idle robot
- `path_planner` returns a valid route on the configured graph
- `robot_agent` executes the route waypoint by waypoint and reports completion
- Planner rejection no longer stalls the queue
- The planner-failure recovery check passes end to end

Task submission with `ros2 service call` could not be verified inside the sandbox because a separate CLI participant remained blocked waiting for service discovery under the same DDS socket restrictions.

This means:

- build and packaging are good
- launch startup is good
- end-to-end demo task submission and completion are verified in a normal shell
- planner rejection recovery is verified in a normal shell
- sandbox restrictions still limit sandbox-only ROS 2 CLI verification

## Current Limitations

- Demo map data is still ROS parameter YAML rather than a richer map format
- No conflict detection or reservation logic
- `fleet_manager` assignment is still simple first-idle selection
- Rejected tasks are dropped after planner failure rather than retried or surfaced through a richer failure channel
- Robot battery is static
- No RViz visualization yet
- Sandbox-only ROS 2 CLI verification remains limited by DDS socket restrictions

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

1. Add conflict reservation and smarter scheduling
2. Add richer task failure reporting for planner-rejected tasks
3. Add visualization or richer runtime introspection

### After that

- Add conflict checking on nodes and edges
- Add priority-aware scheduling
- Add automated tests around manager selection and failure handling
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
ros2 run fleet_bringup check_planner_failure_path.sh
```

Most likely next files to edit:

- `fleet_ws/src/fleet_manager/src/fleet_manager_node.cpp`
- `fleet_ws/src/path_planner/src/path_planner_node.cpp`
- `fleet_ws/src/fleet_bringup/scripts/check_planner_failure_path.sh`
