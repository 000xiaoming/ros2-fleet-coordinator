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

## Documentation Map

Use these project notes as the main documentation set:

- `README.md`: project overview and entry point
- `STATUS.md`: current verified technical state, limitations, and next steps
- `BUGS.md`: all meaningful bugs and environmental blockers
- `PROGRESS.md`: chronological progress log
- `TECHNICAL_APPROACH.md`: architecture, technology choices, reasons, and comparisons with alternatives

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

Submit a demo task from another shell:

```bash
cd /home/bruce/project-a/ros2-fleet-coordinator/fleet_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run fleet_bringup submit_demo_task.sh --task-id demo_001
```

## Current Status

The project is past the initial scaffold stage. The current V1 flow now includes:

- robot state publication from `robot_agent`
- task submission and dispatching in `fleet_manager`
- graph-based route planning in `path_planner`
- simulated waypoint-by-waypoint task execution
- launch-time map configuration for graph topology and waypoint coordinates
- a repeatable demo task submission helper for host-side testing

The main remaining gaps are conflict handling, smarter scheduling, and full end-to-end runtime verification outside the sandbox.

## Technical Rationale

The adopted approach is documented in `TECHNICAL_APPROACH.md`, including:

- why ROS 2, C++, `ament_cmake`, and `colcon` were chosen
- why the system is split into manager, planner, agent, bringup, and interface packages
- why the current planner uses a graph plus BFS
- how those choices compare with alternatives such as Python-only nodes, embedded planning, topic-only request/reply flows, Dijkstra/A*, Nav2-first integration, and custom non-ROS communication

## Update Workflow

Recommended routine after each meaningful operation:

1. Update `STATUS.md` if the current state or next steps changed.
2. Record any encountered bug in `BUGS.md`.
3. Update `TECHNICAL_APPROACH.md` if the architecture or technology rationale changed.
4. Add a short entry to `PROGRESS.md`.
5. If a bug was resolved, commit the related code and notes together.
6. Attempt to push the resolved bugfix to GitHub immediately and record why that submission was made.

Example:

```bash
cd /home/bruce/project-a/ros2-fleet-coordinator
git add -A
git commit -m "Implement route assignment flow"
git push
```
