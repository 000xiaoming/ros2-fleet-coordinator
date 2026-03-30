# Project Status

Date: 2026-03-30

## Goal

Create a standalone ROS 2 C++ project for a multi-robot fleet coordinator inside `/home/bruce/project-a` without affecting other projects.

## Documentation Coverage

The project documentation is split by purpose:

- `README.md`: overview and quick start
- `STATUS.md`: current behavior, verification status, limitations, and next steps
- `BUGS.md`: all meaningful bugs and environmental blockers
- `PROGRESS.md`: chronological record of meaningful project changes
- `SIMULATION_PLAN.md`: staged Gazebo/Nav2/RViz rollout plan
- `TECHNICAL_APPROACH.md`: technical design, chosen technologies, rationale, and comparisons with alternatives

## Project Directory

- Root: `/home/bruce/project-a/ros2-fleet-coordinator`
- Workspace: `/home/bruce/project-a/ros2-fleet-coordinator/fleet_ws`

## What Was Created

The project was initialized as an isolated ROS 2 workspace with these packages:

- `fleet_msgs`
- `fleet_manager`
- `robot_agent`
- `fleet_runtime`
- `path_planner`
- `fleet_bringup`

Main files:

- `README.md`
- `BUGS.md`
- `TECHNICAL_APPROACH.md`
- `fleet_ws/src/fleet_msgs/msg/RobotState.msg`
- `fleet_ws/src/fleet_msgs/msg/Task.msg`
- `fleet_ws/src/fleet_msgs/msg/TaskAssignment.msg`
- `fleet_ws/src/fleet_msgs/msg/TaskStatus.msg`
- `fleet_ws/src/fleet_msgs/srv/SubmitTask.srv`
- `fleet_ws/src/fleet_msgs/srv/PlanRoute.srv`
- `fleet_ws/src/fleet_msgs/action/ExecuteTask.action`
- `fleet_ws/src/fleet_manager/src/fleet_manager_node.cpp`
- `fleet_ws/src/robot_agent/src/robot_agent_node.cpp`
- `fleet_ws/src/fleet_runtime/src/goal_follower_node.cpp`
- `fleet_ws/src/fleet_runtime/src/fleet_visualization_node.cpp`
- `fleet_ws/src/path_planner/src/path_planner_node.cpp`
- `fleet_ws/src/fleet_bringup/launch/demo.launch.py`
- `fleet_ws/src/fleet_bringup/launch/sim_demo.launch.py`
- `fleet_ws/src/fleet_bringup/config/demo_map.yaml`
- `fleet_ws/src/fleet_bringup/worlds/fleet_demo.world`
- `fleet_ws/src/fleet_bringup/urdf/simple_diffbot.urdf`
- `fleet_ws/src/fleet_bringup/rviz/fleet_simulation.rviz`
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
- Supports `execution_mode=external_navigation`, where it publishes waypoint pose goals on a configured topic, consumes external navigation results, and reports terminal `completed` or `failed` robot states without faking route progress locally

### `fleet_manager`

- Subscribes to `robot_states`
- Stores latest state per robot
- Provides `submit_task` service using `fleet_msgs/srv/SubmitTask`
- Queues pending tasks
- Requests a route from `path_planner` before assignment
- Publishes `fleet_msgs/msg/TaskAssignment` after ranking idle robots by pickup proximity and battery, then trying planner requests until one candidate succeeds
- Publishes `fleet_msgs/msg/TaskStatus` on `task_statuses` for queued, waiting, assigned, executing, completed, and failed task lifecycle events
- Drops planner-rejected front-of-queue tasks so one invalid task does not stall the queue
- Reserves assigned route waypoints and postpones conflicting tasks until those reservations clear
- Releases reservations on task completion or robot-state timeout
- Preserves assignment state until the agent reports execution or completion

### `path_planner`

- Provides a `plan_route` service using a parameter-driven waypoint graph
- Returns BFS-planned routes from robot start waypoint to pickup and then dropoff
- Avoids currently reserved intermediate waypoints and reports when a route is blocked by active reservations
- Rejects tasks whose start, pickup, or dropoff waypoints are not on the configured graph
- Loads demo graph topology from `fleet_bringup/config/demo_map.yaml`

### `fleet_runtime`

- `goal_follower_node` subscribes to per-robot goal poses, follows them against Gazebo odometry, publishes `cmd_vel`, and reports `succeeded` or `failed` navigation results
- `goal_follower_node` republishes odometry as a pose estimate so `robot_agent` can publish navigation-backed `RobotState` poses
- `fleet_visualization_node` publishes RViz markers for the waypoint graph, waypoint labels, robot states, and active assigned routes

### `fleet_bringup`

- Launches:
  - `fleet_manager`
  - `path_planner`
  - `robot_agent_1`
  - `robot_agent_2`
  - `sim_demo.launch.py` for Gazebo, two simulated robots, runtime goal followers, and RViz
- Ships:
  - `config/demo_map.yaml` for the demo graph and waypoint coordinates
  - `worlds/fleet_demo.world` for a Gazebo simulation world
  - `urdf/simple_diffbot.urdf` for a simple differential-drive robot model
  - `rviz/fleet_simulation.rviz` for operator visualization
  - `submit_demo_task.sh` for repeatable task submission
  - `check_planner_failure_path.sh` for end-to-end planner-failure recovery verification, including lifecycle status checks on `/task_statuses`
  - `check_conflict_reservation.sh` for end-to-end reservation-conflict verification

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

### 5. Async planner callback stale-state fix

Problem:
`fleet_manager` could receive a planner response after the selected robot had timed out and been removed from manager state. The callback also mutated the queue by assuming the planned task was still at `pending_tasks_.front()`.

Fix:

- Updated `fleet_manager` to find the queued task by `task_id` before rotating or erasing it
- Guarded assignment so planner success only proceeds if the selected robot is still tracked and still idle
- Left the task queued when planner success arrives for a robot that is no longer eligible for assignment

### 6. Multi-candidate scheduler upgrade

Problem:
The manager still treated scheduling as a single-shot decision. It ranked one idle robot, asked the planner once, and either assigned that robot or delayed/failed the task without checking whether another idle robot could complete it.

Fix:

- Updated `fleet_manager` to rank all idle robots by pickup proximity with battery as a tie-breaker
- Added a planning-attempt state machine so the manager retries the same task across alternate idle robots before postponing or failing it
- Only publishes `waiting` when every candidate is blocked by reservations; otherwise it assigns the first candidate with a valid route

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
colcon build --packages-select fleet_manager
```

Additional 2026-03-21 and 2026-03-23 verification:

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
ros2 run fleet_bringup check_conflict_reservation.sh
```

Result:

- `submit_demo_task.sh` is runnable through `ros2 run`
- `submit_task` accepts a demo task
- `fleet_manager` queues the task and assigns it to an idle robot
- `path_planner` returns a valid route on the configured graph
- `robot_agent` executes the route waypoint by waypoint and reports completion
- Planner rejection no longer stalls the queue
- `task_statuses` now reports queued, waiting, assigned, executing, completed, and failed lifecycle events
- The planner-failure recovery check passes end to end
- Conflicting tasks are postponed while route waypoints are reserved and resume after reservations clear
- The reservation-conflict recovery check passes end to end
- The `fleet_manager` async planner callback no longer depends on stale queue-front or stale robot-state assumptions during assignment
- `robot_agent` now has an external-navigation integration mode that publishes pose goals and waits for external navigation results instead of always auto-completing locally

Task submission with `ros2 service call` could not be verified inside the sandbox because a separate CLI participant remained blocked waiting for service discovery under the same DDS socket restrictions.

Additional 2026-03-30 verification:

```bash
cd /home/bruce/project-a/ros2-fleet-coordinator/fleet_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_agent fleet_bringup
colcon build --packages-select fleet_manager
colcon build --packages-select fleet_runtime fleet_bringup robot_agent fleet_manager
ros2 launch fleet_bringup sim_demo.launch.py --show-args
timeout 20s ros2 launch fleet_bringup sim_demo.launch.py gui:=false use_rviz:=false
```

This means:

- build and packaging are good
- launch startup is good
- the Gazebo-based simulation launch resolves against the installed workspace
- a headless simulation run starts `gzserver`, the fleet nodes, robot publishers, and robot spawn processes
- end-to-end demo task submission and completion are verified in a normal shell
- planner rejection recovery is verified in a normal shell
- reservation conflict handling is verified in a normal shell
- sandbox restrictions still limit sandbox-only ROS 2 CLI verification

## Current Limitations

- Demo map data is still ROS parameter YAML rather than a richer map format
- Conflict handling is waypoint-level only; there is no edge reservation or time-window model
- `fleet_manager` assignment is distance-ranked across idle robots, but not yet priority- or ETA-aware
- Rejected tasks are dropped after planner failure rather than retried
- `task_statuses` is a live topic stream only; there is no persisted task history or query API
- Robot battery is static
- RViz visualization is now configured through `fleet_visualization_node` and `fleet_simulation.rviz`, but full operator validation with live simulated motion is still pending
- Gazebo bringup is present, but full robot spawn plus task-driven motion was not observed within the 20-second headless verification window
- Gazebo is launched by `sim_demo.launch.py`, but Nav2 is still not installed in this environment
- Direct `NavigateToPose` action integration remains blocked in this environment because the `nav2_msgs` development package is unavailable at build time
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

1. Add a namespaced navigation bridge from `fleet_goal_pose` and `fleet_navigation_result` to each robot's Nav2 stack
2. Bring up Gazebo plus per-robot localization/Nav2 so `robot_agent` external-navigation mode drives real simulated motion
3. Add RViz markers and overlays for goals, routes, reservations, and task status
4. Add bounded retry/reassignment logic for navigation failures
5. Add automated demo scripts and tests around failure/recovery and queue behavior

### After that

- Add edge-level and time-aware conflict checking
- Add ETA- and priority-aware scheduling
- Add persisted task history or queryable operator APIs

The detailed staged rollout is documented in `SIMULATION_PLAN.md`.

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
