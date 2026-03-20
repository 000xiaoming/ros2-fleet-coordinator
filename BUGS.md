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
