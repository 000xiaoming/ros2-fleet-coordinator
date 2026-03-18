# ROS 2 Fleet Coordinator

`ros2-fleet-coordinator` is a standalone ROS 2 C++ workspace for a multi-robot fleet coordination project. It is intentionally isolated from the rest of `/home/bruce/project-a`.

## Goal

Build a graph-based fleet management system where multiple robots:

- report their state
- receive tasks from a central coordinator
- follow waypoint routes
- avoid basic route conflicts

## Workspace Layout

```text
ros2-fleet-coordinator/
├── .gitignore
├── README.md
└── fleet_ws/
    └── src/
        ├── fleet_bringup/
        ├── fleet_manager/
        ├── fleet_msgs/
        ├── path_planner/
        └── robot_agent/
```

## Packages

- `fleet_msgs`: custom messages, services, and actions for fleet coordination
- `fleet_manager`: central dispatcher node
- `robot_agent`: per-robot execution node
- `path_planner`: graph route planner node
- `fleet_bringup`: launch files and future config

## Proposed V1 Scope

1. Submit a transport task with pickup and dropoff waypoint IDs.
2. Track available robots and assign one task at a time.
3. Generate a waypoint path on a graph map.
4. Simulate progress and publish status updates.
5. Visualize state in RViz or simple console logs.

## Next Steps

1. Install and source a ROS 2 distribution such as Humble or Jazzy.
2. From `fleet_ws`, run `colcon build`.
3. Source `install/setup.bash`.
4. Launch the starter stack.

```bash
cd /home/bruce/project-a/ros2-fleet-coordinator/fleet_ws
colcon build
source install/setup.bash
ros2 launch fleet_bringup demo.launch.py
```

## Current Status

This scaffold provides the project structure and starter nodes. The business logic is still intentionally minimal so we can extend it cleanly in the next step.

## Update Workflow

Use these project notes for ongoing GitHub updates:

- `STATUS.md`: current technical state, verified behavior, blockers, and next steps
- `PROGRESS.md`: chronological progress log for meaningful changes

Recommended routine after each meaningful operation:

1. Update `STATUS.md` if the current state or next steps changed.
2. Add a short entry to `PROGRESS.md`.
3. Commit the related code and notes together.
4. Push to GitHub.

Example:

```bash
cd /home/bruce/project-a/ros2-fleet-coordinator
git add -A
git commit -m "Implement route assignment flow"
git push
```
