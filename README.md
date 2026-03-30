# ROS 2 Fleet Coordinator

`ros2-fleet-coordinator` is a standalone ROS 2 C++ multi-robot fleet coordination project built inside `/home/bruce/project-a` without coupling to other workspaces.

The project is centered on one end-to-end question:

How do multiple robots accept transport tasks, get assigned by a central dispatcher, follow planned routes, avoid simple route conflicts, expose runtime state clearly, and evolve from a logic demo into a real simulation-backed system?

## Project Overview

This repository implements a modular fleet stack with:

- task intake through a central `fleet_manager`
- graph-based route planning through `path_planner`
- per-robot execution handling through `robot_agent`
- shared ROS 2 interfaces in `fleet_msgs`
- simulation execution helpers and RViz markers in `fleet_runtime`
- launch, config, Gazebo, and RViz assets in `fleet_bringup`

The current system is beyond an initial scaffold. It already supports task submission, planner-backed assignment, conflict-aware postponement, lifecycle status reporting, Gazebo-oriented runtime hooks, and simulation visualization.

## What The System Does Today

### Fleet coordination

- tracks robot state on `robot_states`
- accepts tasks through `submit_task`
- ranks idle robots by pickup proximity with battery as a tie-breaker
- requests a planned route before assignment
- retries alternate idle robots when one candidate cannot take a task
- postpones tasks when reserved waypoints block the route
- releases reservations when a task completes, fails, or a robot times out

### Planning

- uses a waypoint graph loaded from config
- plans `start -> pickup -> dropoff` routes with BFS
- rejects invalid waypoint requests
- reports when a route is blocked by reserved waypoints

### Task lifecycle visibility

- publishes `queued`
- publishes `waiting`
- publishes `assigned`
- publishes `executing`
- publishes `completed`
- publishes `failed`

### Simulation/runtime support

- supports a pure logic demo flow through `demo.launch.py`
- supports a Gazebo-oriented simulation flow through `sim_demo.launch.py`
- spawns two simple differential-drive robots in Gazebo
- publishes RViz markers for waypoint graph, routes, and robot state
- provides an external-navigation execution mode so `robot_agent` no longer has to fake all execution locally

## Current Architecture

```text
task client
   |
   v
submit_task service
   |
   v
fleet_manager ---------------------------> task_statuses
   |   \
   |    \ async plan request
   |     \
   v      v
task_assignments   path_planner
   |
   v
robot_agent
   |
   +--> simulated internal execution
   |
   +--> external navigation execution
          |
          v
      fleet_runtime/goal_follower
          |
          v
        Gazebo robot motion
```

## Packages

- `fleet_msgs`
  Shared messages, services, and actions used across manager, planner, runtime, and robot-side execution.
- `fleet_manager`
  Central dispatch node responsible for task queueing, robot selection, reservation tracking, and assignment publishing.
- `robot_agent`
  Robot-side execution node that consumes assignments, publishes `RobotState`, and can run in `simulated` or `external_navigation` mode.
- `path_planner`
  Graph-based planning service that generates waypoint routes and enforces reservation-aware planning constraints.
- `fleet_runtime`
  Runtime support package for simulation, including a simple goal follower and an RViz marker publisher.
- `fleet_bringup`
  Launch files, config, Gazebo world, URDF, and RViz setup for both the logic demo and the simulation demo.

## Workspace Layout

```text
ros2-fleet-coordinator/
├── README.md
├── STATUS.md
├── BUGS.md
├── PROGRESS.md
├── SIMULATION_PLAN.md
├── TECHNICAL_APPROACH.md
└── fleet_ws/
    └── src/
        ├── fleet_bringup/
        ├── fleet_manager/
        ├── fleet_msgs/
        ├── fleet_runtime/
        ├── path_planner/
        └── robot_agent/
```

## Latest Progress

Recent project milestones include:

- planner-backed task assignment instead of internal bookkeeping only
- queue-stall recovery when an invalid task reaches the front of the queue
- explicit task lifecycle reporting through `task_statuses`
- waypoint reservation and conflict postponement
- multi-candidate robot selection instead of one-shot scheduling
- async planner callback hardening around stale queue and stale robot state
- external navigation mode in `robot_agent`
- Gazebo simulation bringup with two simple robots
- RViz marker visualization for route graph, robots, and assigned paths

For the full chronological log, see [`PROGRESS.md`](./PROGRESS.md).

## Verified State

The repository has verified:

- package builds for `fleet_msgs`, `fleet_manager`, `robot_agent`, `path_planner`, `fleet_runtime`, and `fleet_bringup`
- logic-demo launch resolution and task helper flow
- planner failure recovery checks
- reservation conflict recovery checks
- simulation launch resolution for `sim_demo.launch.py`
- headless startup of `gzserver`, fleet nodes, robot publishers, and spawn processes

The main remaining runtime gap is longer host-side validation of full Gazebo robot spawning and motion, followed by direct Nav2 integration when Nav2 packages are available in the environment.

## Quick Start

### Build

```bash
cd /home/bruce/project-a/ros2-fleet-coordinator/fleet_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fleet_msgs fleet_manager robot_agent path_planner fleet_runtime fleet_bringup
source install/setup.bash
```

### Logic demo

```bash
cd /home/bruce/project-a/ros2-fleet-coordinator/fleet_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fleet_bringup demo.launch.py
```

From another shell:

```bash
cd /home/bruce/project-a/ros2-fleet-coordinator/fleet_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run fleet_bringup submit_demo_task.sh --task-id demo_001
```

### Simulation demo

```bash
cd /home/bruce/project-a/ros2-fleet-coordinator/fleet_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/home/bruce/project-a/ros2-fleet-coordinator/fleet_ws/log/ros
ros2 launch fleet_bringup sim_demo.launch.py
```

Headless check:

```bash
cd /home/bruce/project-a/ros2-fleet-coordinator/fleet_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/home/bruce/project-a/ros2-fleet-coordinator/fleet_ws/log/ros
timeout 20s ros2 launch fleet_bringup sim_demo.launch.py gui:=false use_rviz:=false
```

## Current Limitations

- direct Nav2 integration is still blocked in this environment because `nav2_msgs` and Nav2 packages are not installed
- the current planner is graph-based and waypoint-level, not continuous-costmap planning
- conflict handling is waypoint reservation only, not edge/time-window coordination
- task history is live-topic-based rather than persisted/queryable
- battery is static and not part of execution realism yet
- full Gazebo spawn-and-motion validation still needs longer host-side runtime testing

## Roadmap

The next major steps are:

1. finish full Gazebo runtime validation for robot spawning and task-driven motion
2. replace the temporary goal-following bridge with direct Nav2 action integration
3. make task failure and recovery more explicit with bounded retry/reassignment logic
4. improve scheduling with ETA- and priority-aware behavior
5. keep expanding RViz/operator visibility and runtime debugging support

The staged simulation plan is tracked in [`SIMULATION_PLAN.md`](./SIMULATION_PLAN.md).

## Documentation Map

- [`STATUS.md`](./STATUS.md)
  Current verified behavior, limitations, and next steps.
- [`BUGS.md`](./BUGS.md)
  Source of truth for meaningful bugs and environmental blockers.
- [`PROGRESS.md`](./PROGRESS.md)
  Chronological progress log.
- [`SIMULATION_PLAN.md`](./SIMULATION_PLAN.md)
  Staged Gazebo/Nav2/RViz rollout plan.
- [`TECHNICAL_APPROACH.md`](./TECHNICAL_APPROACH.md)
  Architecture and technology rationale.

## Why This Project Is Interesting

This is not just a ROS 2 package skeleton. The project already covers several real robotics-system concerns in one codebase:

- asynchronous planner/dispatcher coordination
- multi-robot task allocation
- route conflict handling
- task lifecycle observability
- simulation/runtime separation
- incremental migration path from logic demo to real navigation-backed execution

That makes it a strong base for further work in fleet orchestration, robot simulation, or navigation-system integration.
