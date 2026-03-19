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
