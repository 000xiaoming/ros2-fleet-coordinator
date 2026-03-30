# Simulation Plan

Date: 2026-03-30

## Goal

Turn the current waypoint-level demo into a real simulated robot system where:

- `fleet_manager` dispatches executable navigation work
- robots move in Gazebo
- RViz shows live fleet/runtime state
- task failures and recovery are visible end to end

## Current Baseline

The repo already has:

- task submission in `fleet_manager`
- graph-based route planning in `path_planner`
- task assignment and lifecycle status publication
- reservation-aware scheduling
- a `robot_agent` that can now run in two modes:
  - `simulated`: existing timer-driven waypoint progression
  - `external_navigation`: publishes waypoint pose goals and waits for an external navigation result instead of faking completion
- a `fleet_runtime` package with:
  - a simple Gazebo-side goal follower
  - an RViz marker publisher for graph, robot, and route state
- a Gazebo simulation launch, world, robot model, and RViz config

## Blocking Environment Gap

Direct `nav2_msgs/action/NavigateToPose` integration is not buildable in the current environment because the `nav2_msgs` development package is not installed. The next bridge to real Nav2 should be implemented once that dependency is available.

## Step-By-Step Execution Plan

### 1. Stand up the simulation runtime

- Add a Gazebo world with two robots and deterministic start poses
- Bring up robot-local localization, TF, and Nav2 stacks
- Enable `use_sim_time` throughout the fleet workspace

Done when:

- both robots spawn in Gazebo
- both robots have namespaced pose topics and navigation stacks
- RViz can display map, TF, and robot motion

### 2. Add a navigation bridge

- Connect each robot's `fleet_goal_pose` topic to its Nav2 `NavigateToPose` action
- Publish `succeeded`, `failed`, `aborted`, or `canceled` on `fleet_navigation_result`
- Keep this bridge robot-namespaced so `robot_agent` stays fleet-facing and Nav2 stays robot-facing

Done when:

- assigning a task causes a real Nav2 goal request
- robot movement in Gazebo corresponds to waypoint goals from `fleet_manager`

### 3. Replace waypoint-only visibility with operator visibility

- Publish RViz markers for pickup/dropoff points, waypoint graph, and reserved waypoints
- Publish current assigned route per robot
- Keep `task_statuses` visible in runtime tooling

Done when:

- an operator can see robot positions, current goals, reservations, and task state in RViz without reading logs

### 4. Make failures explicit

- Surface planner rejection, navigation failure, timeout, and robot-loss conditions as explicit task events
- Keep failure reason strings machine-readable and human-readable

Done when:

- each failure mode is visible on `task_statuses`
- terminal robot states release reservations cleanly

### 5. Add bounded recovery

- Retry transient navigation failures on the same robot
- Requeue or reassign work when a robot fails before completing the route
- Keep a bounded retry count to avoid infinite loops

Done when:

- at least one repeatable demo shows reassignment or retry after a failure

### 6. Automate the demos

- Add scripts for:
  - happy-path task execution
  - navigation failure and recovery
  - reservation conflict and eventual completion
- Add package-level build/test checks around scheduler and robot state transitions

Done when:

- the simulation demos are repeatable from launch scripts
- core integration checks can be rerun after code changes

## Immediate Next Coding Step

Verify full Gazebo robot spawning and task-driven motion outside the current short headless timeout window, then replace the temporary goal follower with a namespaced bridge to each robot's Nav2 action server once the Nav2 development packages are available in the environment.
