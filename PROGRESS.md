# Progress Log

## How To Use

Add one short entry after each meaningful project step.

Keep each entry focused on:

- what changed
- what was verified
- what remains next

Suggested format:

```text
## YYYY-MM-DD

### Short title
- Changed:
- Verified:
- Next:
```

## 2026-03-18

### Project initialization and GitHub setup
- Changed:
  - Created standalone ROS 2 C++ project `ros2-fleet-coordinator`
  - Added packages: `fleet_msgs`, `fleet_manager`, `robot_agent`, `path_planner`, `fleet_bringup`
  - Implemented first V1 coordination flow:
    - `robot_agent` publishes `RobotState`
    - `fleet_manager` subscribes to robot states
    - `fleet_manager` exposes `submit_task`
    - `fleet_manager` queues tasks and assigns the next task to an idle robot
  - Fixed package metadata so all packages are detected as `ros.ament_cmake`
  - Initialized git repo, renamed branch to `main`, connected GitHub remote, and pushed first commit
- Verified:
  - `colcon list` detects all five packages correctly
  - `colcon build --cmake-clean-cache --packages-select fleet_msgs fleet_manager robot_agent path_planner fleet_bringup` succeeds
  - SSH auth to GitHub works for account `000xiaoming`
  - `git push -u origin main` succeeds
- Next:
  - Implement route generation in `path_planner`
  - Connect `fleet_manager` to planner output before assignment
  - Add simulated task execution and completion feedback in `robot_agent`

## 2026-03-19

### Route planning and assignment flow
- Changed:
  - Added repo-local bug tracking in `BUGS.md`
  - Added `fleet_msgs/TaskAssignment` and `fleet_msgs/PlanRoute`
  - Extended `RobotState` with `current_waypoint`
  - Implemented a small graph-backed route planner service in `path_planner`
  - Updated `fleet_manager` to request routes before publishing assignments
  - Updated `robot_agent` to accept assignments and simulate waypoint progress
  - Added launch parameters for each robot's starting waypoint
- Verified:
  - `colcon build --packages-select fleet_msgs fleet_manager robot_agent path_planner fleet_bringup` succeeds
  - `ros2 launch fleet_bringup demo.launch.py` starts all nodes with the new interfaces loaded
  - `fleet_manager` logs that it tracks both robots and `robot_agent` logs waypoint-aware idle states
  - Separate `ros2 service call` verification is still blocked by sandbox DDS socket restrictions
- Next:
  - Move the waypoint graph and coordinates into config files
  - Add a repeatable task submission helper for host-side runtime testing
  - Verify full task assignment and completion flow outside the sandbox

## 2026-03-20

### Documentation expansion for bugs and technical rationale
- Changed:
  - Added `TECHNICAL_APPROACH.md`
  - Updated `README.md` to point to the full documentation set
  - Updated `STATUS.md` to describe documentation coverage and summarize the implementation approach
  - Updated `BUGS.md` to state that all meaningful bugs and environmental blockers must be recorded there
- Verified:
  - Reviewed all repository documentation files for consistent wording and cross-references
- Next:
  - Keep `TECHNICAL_APPROACH.md` updated as planner configuration, scheduling, and conflict-handling evolve

### Externalized demo map configuration
- Changed:
  - Added `fleet_bringup/config/demo_map.yaml` for waypoint coordinates and graph edges
  - Updated `path_planner` to load graph topology from ROS 2 parameters instead of a hardcoded map
  - Updated `robot_agent` to load waypoint positions from ROS 2 parameters with a built-in fallback
  - Updated `demo.launch.py` and `fleet_bringup` packaging to distribute and load the shared config
- Verified:
  - `source /opt/ros/humble/setup.bash && colcon build --packages-select robot_agent path_planner fleet_bringup`
- Next:
  - Add a repeatable task submission helper for host-side runtime testing
  - Verify full task assignment and completion flow outside the sandbox

### Repeatable task submission helper
- Changed:
  - Added `fleet_bringup/scripts/submit_demo_task.sh`
  - Installed the script with `fleet_bringup` so it can be run via `ros2 run`
  - Updated project docs to include the helper in quick-start and current status
- Verified:
  - `bash -n fleet_ws/src/fleet_bringup/scripts/submit_demo_task.sh`
- Next:
  - Verify full task assignment and completion flow outside the sandbox using the helper

## 2026-03-21

### End-to-end demo flow verified
- Changed:
  - Fixed executable permissions on `fleet_ws/src/fleet_bringup/scripts/submit_demo_task.sh`
  - Rebuilt `fleet_bringup` so the helper is installed as a runnable program
  - Verified the documented `ros2 run fleet_bringup submit_demo_task.sh` workflow against the live demo stack
- Verified:
  - `source /opt/ros/humble/setup.bash && colcon build --packages-select fleet_msgs fleet_manager robot_agent path_planner fleet_bringup`
  - `bash -n fleet_ws/src/fleet_bringup/scripts/submit_demo_task.sh`
  - `timeout 12s ros2 launch fleet_bringup demo.launch.py`
  - `ros2 run fleet_bringup submit_demo_task.sh --task-id demo_001`
  - Observed `fleet_manager` queue the task, `path_planner` return a route, `robot_agent_1` execute the route, and completion return the robot to idle
- Next:
  - Add tests for planner failure paths and manager queue behavior
  - Implement conflict handling and smarter scheduling beyond first-idle dispatch

### Queue-stall fix and recovery check
- Changed:
  - Updated `fleet_manager` to drop planner-rejected front-of-queue tasks instead of leaving them to block later tasks
  - Added `fleet_bringup/scripts/check_planner_failure_path.sh` as a runtime regression check
  - Installed the new check script with `fleet_bringup` so it can be run through `ros2 run`
- Verified:
  - `source /opt/ros/humble/setup.bash && colcon build --event-handlers console_direct+ --packages-select fleet_manager`
  - `bash -n fleet_ws/src/fleet_bringup/scripts/check_planner_failure_path.sh`
  - `source /opt/ros/humble/setup.bash && colcon build --packages-select fleet_bringup`
  - `source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run fleet_bringup check_planner_failure_path.sh`
- Next:
  - Add richer task failure reporting for rejected tasks
  - Add conflict reservation and smarter scheduling
