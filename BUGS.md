# Bug Log

Track every meaningful bug encountered during development here.

This file is the source of truth for bug documentation in this repository, including:

- code defects
- integration failures
- build/package issues
- environment blockers that materially affect verification

For each bug:

- record the date
- describe the observed behavior and root cause
- mark whether it is `open` or `resolved`
- note how it was verified
- explain whether the problem is code-related or environmental
- after resolution, attempt to submit the fix to GitHub and record the result

Suggested format:

```text
## YYYY-MM-DD - Short title
- Status:
- Observed:
- Root cause:
- Fix:
- Verified:
- GitHub submission:
```

## 2026-03-23 - Async planner callback used stale robot and queue state
- Status: resolved
- Observed:
  - `fleet_manager` could finish an async `plan_route` request after the selected robot had already timed out and been removed from `robot_states_`.
  - The callback also rotated or erased `pending_tasks_.begin()` instead of locating the originally planned task by `task_id`, which made queue handling depend on the front entry staying unchanged while planning was in flight.
- Root cause:
  - The planner response handler assumed both robot liveness and queue front identity were still valid when the async future completed.
- Fix:
  - Updated `fleet_ws/src/fleet_manager/src/fleet_manager_node.cpp` to find the queued task by `task_id` before rotating or erasing it.
  - Guarded assignment so planner success only proceeds if the selected robot is still tracked and still idle.
  - Left the task queued when the planned-for robot disappeared or changed state before the planner reply arrived.
- Verified:
  - `colcon build --packages-select fleet_manager`
  - `source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_LOG_DIR=/home/bruce/project-a/ros2-fleet-coordinator/fleet_ws/log/ros && ros2 run fleet_bringup check_planner_failure_path.sh`
  - `source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_LOG_DIR=/home/bruce/project-a/ros2-fleet-coordinator/fleet_ws/log/ros && ros2 run fleet_bringup check_conflict_reservation.sh`
  - Verified that both end-to-end regression scripts still pass after the `fleet_manager` change
- GitHub submission:
  - Pending until the fix is committed and a push attempt is made.

## 2026-03-30 - Direct Nav2 action integration blocked by missing development package
- Status: open
- Observed:
  - Building `robot_agent` with a direct `nav2_msgs/action/NavigateToPose` client failed during CMake configure.
  - The environment does not provide `nav2_msgsConfig.cmake`, so the package cannot currently compile against Nav2 action headers.
- Root cause:
  - The current ROS installation in this workspace does not include the `nav2_msgs` development package needed for direct Nav2 action integration.
- Fix:
  - Reworked `robot_agent` to expose an `external_navigation` execution mode that publishes pose goals and consumes navigation result topics without fabricating local task completion.
  - Kept the direct Nav2 bridge as the next step once the missing Nav2 development dependency is installed.
- Verified:
  - `colcon build --packages-select robot_agent fleet_bringup`
  - Verified that the adjusted `robot_agent` and bringup packages compile without `nav2_msgs`
- GitHub submission:
  - Pending until the fix is committed and a push attempt is made.

## 2026-03-30 - Headless Gazebo verification did not reach entity spawn within short timeout
- Status: open
- Observed:
  - `timeout 20s ros2 launch fleet_bringup sim_demo.launch.py gui:=false use_rviz:=false` starts `gzserver`, `fleet_manager`, `path_planner`, `fleet_visualization`, both `robot_agent` nodes, both `robot_state_publisher` nodes, and both `spawn_entity.py` processes.
  - During that 20-second window, both spawn processes remained waiting for the `/spawn_entity` service and full robot insertion into Gazebo was not observed.
- Root cause:
  - Not confirmed yet. This may be a slow Gazebo startup path or an environment/runtime issue rather than a code defect in the launch wiring.
- Fix:
  - No code fix yet. The current next step is longer host-side verification and direct inspection of Gazebo service availability outside the short timeout window.
- Verified:
  - `timeout 20s ros2 launch fleet_bringup sim_demo.launch.py gui:=false use_rviz:=false`
  - Verified that the fleet stack starts and that the blocker appears specifically at the Gazebo entity-spawn stage
- GitHub submission:
  - Not submitted as a resolved fix because the issue remains under investigation.


## 2026-03-19 - Task assignment stopped at manager bookkeeping
- Status: resolved
- Observed:
  - Submitted tasks could be queued and marked assigned inside `fleet_manager`, but no planner route was requested, no assignment was sent to agents, and no execution/completion lifecycle existed.
- Root cause:
  - The initial scaffold only implemented internal task queue handling. The route-planning interface, assignment transport, and agent execution path were missing.
- Fix:
  - Added `fleet_msgs/TaskAssignment` and `fleet_msgs/PlanRoute`
  - Extended `RobotState` with `current_waypoint`
  - Implemented graph-backed route planning in `path_planner`
  - Updated `fleet_manager` to request routes and publish assignments
  - Updated `robot_agent` to consume assignments and simulate progress/completion
- Verified:
  - `colcon build --packages-select fleet_msgs fleet_manager robot_agent path_planner fleet_bringup`
  - `ros2 launch fleet_bringup demo.launch.py`
- GitHub submission:
  - Pending until the fix is committed and a push attempt is made.

## 2026-03-19 - DDS socket restrictions block CLI runtime verification in sandbox
- Status: open
- Observed:
  - ROS 2 nodes can start, but DDS transport reports socket permission errors in the sandbox. A separate `ros2 service call` client remained blocked waiting for service discovery.
- Root cause:
  - The current execution environment restricts network/socket operations required by the DDS transport and CLI discovery path.
- Fix:
  - No code fix yet. This is currently an environment limitation rather than an application logic defect.
- Verified:
  - Reproduced while running `ros2 launch fleet_bringup demo.launch.py`
  - Reproduced while attempting `ros2 service call /submit_task ...`
- GitHub submission:
  - Not submitted as a code fix because the issue is environmental and remains unresolved.

## 2026-03-21 - Demo helper script was not executable
- Status: resolved
- Observed:
  - The documented helper existed at `fleet_ws/src/fleet_bringup/scripts/submit_demo_task.sh`, but direct execution failed with `Permission denied`.
  - The installed helper path was not usable until the source script permission bit was corrected and the package was rebuilt.
- Root cause:
  - The source script was missing the executable bit even though `fleet_bringup/CMakeLists.txt` installs it with `install(PROGRAMS ...)`.
- Fix:
  - Marked `fleet_ws/src/fleet_bringup/scripts/submit_demo_task.sh` executable and rebuilt `fleet_bringup`.
- Verified:
  - `colcon build --packages-select fleet_bringup`
  - `ros2 run fleet_bringup submit_demo_task.sh --task-id demo_001`
  - Verified live demo flow: task queued, route planned, task assigned, robot executed waypoints, and completion published
- GitHub submission:
  - Pending until the fix is committed and a push attempt is made.

## 2026-03-21 - Planner rejection stalled the manager queue
- Status: resolved
- Observed:
  - If an invalid task reached the front of `pending_tasks_`, `fleet_manager` logged the planner rejection but kept that task at the front of the queue.
  - A later valid task could be accepted by `submit_task` but never assigned because the invalid front task blocked progress.
- Root cause:
  - The planner failure branch in `fleet_manager` returned without removing the rejected front-of-queue task.
- Fix:
  - Updated `fleet_manager` to remove planner-rejected front-of-queue tasks.
  - Added explicit `TaskStatus` reporting so rejected tasks publish a machine-readable `failed` event on `task_statuses`.
  - Added `fleet_ws/src/fleet_bringup/scripts/check_planner_failure_path.sh` to verify that a valid task still runs after an invalid task is rejected.
- Verified:
  - `colcon build --event-handlers console_direct+ --packages-select fleet_manager`
  - `bash -n fleet_ws/src/fleet_bringup/scripts/check_planner_failure_path.sh`
  - `colcon build --packages-select fleet_bringup`
  - `ros2 run fleet_bringup check_planner_failure_path.sh`
  - Verified that `task_statuses` reports the rejection reason and that a later valid task is still assigned and completed
- GitHub submission:
  - Pending until the fix is committed and a push attempt is made.

## 2026-03-22 - Reservation recovery reassignment race
- Status: resolved
- Observed:
  - `check_conflict_reservation.sh` initially showed the second task remain stuck after the first task completed.
  - `fleet_manager` reassigned the just-finished robot immediately after the completion update, but `robot_agent` still rejected the new assignment as busy.
- Root cause:
  - `robot_agent` published a `completed` state one timer tick before clearing its internal `active_task_` flag. That left a one-cycle race where `fleet_manager` correctly saw the robot as available while the agent still considered itself busy.
- Fix:
  - Updated `fleet_ws/src/robot_agent/src/robot_agent_node.cpp` to clear the active task immediately after publishing the terminal `completed` state.
  - Kept the `completed` state publication intact so `fleet_manager` can still release reservations and publish lifecycle updates.
- Verified:
  - `colcon build --packages-select robot_agent fleet_bringup`
  - `ros2 run fleet_bringup check_conflict_reservation.sh`
  - Verified that the second conflicting task reaches `waiting`, then `assigned`, `executing`, and `completed` after the first task clears its reservation.
- GitHub submission:
  - Pending until the fix is committed and a push attempt is made.
