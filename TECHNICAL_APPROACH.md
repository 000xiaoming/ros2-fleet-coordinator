# Technical Approach

Date: 2026-03-30

## Goal

Document the technical approach used in `ros2-fleet-coordinator`, why each major technology choice was made, and how those choices compare with reasonable alternatives for this project stage.

## System Approach

The project uses a modular ROS 2 architecture with six packages:

- `fleet_msgs`: shared message and service contracts
- `fleet_manager`: central task intake and dispatch logic
- `robot_agent`: per-robot execution coordination
- `fleet_runtime`: simulation-side goal execution and visualization support
- `path_planner`: graph-based route planning service
- `fleet_bringup`: launch-time composition and configuration entry point

The current design target is a simulated, graph-based fleet coordinator that is simple enough to verify incrementally, but structured enough to evolve into a more realistic multi-robot system.

## Chosen Technical Stack

### ROS 2

Chosen because:

- The project is explicitly a robotics coordination system, and ROS 2 is the standard integration layer for nodes, topics, services, launch, and package structure.
- It provides the communication patterns this project needs now: state publication, task submission, route planning requests, and launch orchestration.
- It keeps the project aligned with common robotics deployment patterns and future integration paths.

Compared with alternatives:

- Compared with a plain custom TCP/HTTP application:
  - ROS 2 is better for robotics-native communication and package organization.
  - A custom network stack would add infrastructure work before solving fleet logic.
- Compared with ROS 1:
  - ROS 2 is the stronger long-term choice for new work because it has better modern support and a more current middleware model.
  - ROS 1 would be a poor choice for a new standalone project unless legacy compatibility were required.

### C++

Chosen because:

- ROS 2 core systems work commonly uses C++ for nodes that may later need predictable performance and closer control over execution behavior.
- It keeps the implementation close to the lower-level runtime patterns used in many production robotics systems.
- It avoids rewriting core nodes later if the project grows from simulation into heavier runtime behavior.

Compared with alternatives:

- Compared with Python:
  - Python would be faster for initial prototyping.
  - C++ is a better fit when the likely direction is more performance-sensitive robotics logic and stronger static structure.
- Compared with mixed-language core logic:
  - A mixed stack increases cognitive load and packaging complexity this early.
  - A single-language core is simpler for a small project.

### `ament_cmake` + `colcon`

Chosen because:

- They are the standard ROS 2 C++ build and workspace tools.
- They support package isolation, interface generation, and iterative package builds cleanly.
- They match the package structure already used in the workspace.

Compared with alternatives:

- Compared with ad hoc CMake or custom scripts:
  - `ament_cmake` and `colcon` integrate better with ROS 2 conventions and generated interfaces.
- Compared with a monorepo-style non-ROS build:
  - A non-ROS build would fight the framework instead of using it.

### Custom messages and services in `fleet_msgs`

Chosen because:

- The project needs explicit shared contracts between manager, planner, and agents.
- A dedicated interface package keeps message/service definitions centralized and versionable.
- It reduces coupling between node implementations.

Compared with alternatives:

- Compared with reusing only generic ROS message types:
  - Generic messages would make the domain model less clear and push implicit assumptions into node code.
- Compared with embedding service/topic schemas directly into each package:
  - A shared interface package is cleaner and avoids duplication.

### Service-based route planning

Chosen because:

- Route planning is naturally request/response in the current design.
- `fleet_manager` needs a route before publishing an assignment, so a service interface is a direct fit.
- It keeps planner logic isolated from dispatch logic.

Compared with alternatives:

- Compared with planning directly inside `fleet_manager`:
  - A separate planner keeps responsibilities cleaner and supports later replacement.
- Compared with a topic-only planner:
  - A topic-based request/reply flow is more awkward for synchronous planning dependencies.
- Compared with ROS 2 actions:
  - Actions are better for long-running, cancelable operations.
  - Current route planning is short-lived and does not yet need action semantics.

### Topic-based robot state updates and assignments

Chosen because:

- Robot state is periodic and naturally modeled as a topic stream.
- Assignments are one-to-many transport over the ROS graph and fit a publish/subscribe model.
- It matches common fleet-style state dissemination patterns.

Compared with alternatives:

- Compared with services for robot state:
  - Polling state via services is less natural and less reactive.
- Compared with actions for assignments:
  - Actions may be appropriate later for richer task lifecycle control, cancellation, and feedback.
  - For the current simulated flow, topic-driven assignment plus state feedback is simpler.

### External navigation bridge before direct Nav2 integration

Chosen because:

- The current environment does not provide the `nav2_msgs` development package, so direct `NavigateToPose` action integration is not buildable yet.
- The project still needs a clean handoff point between fleet coordination and robot-local execution.
- Publishing pose goals and consuming navigation result topics keeps `robot_agent` aligned with a future Nav2-backed runtime while still allowing Gazebo-side motion now.

Compared with alternatives:

- Compared with keeping fully fake timer-based execution:
  - The bridge approach moves the system closer to a real simulated robot stack.
  - It makes robot motion and task completion depend on an external runtime rather than local task auto-complete.
- Compared with blocking all simulation work until Nav2 is installed:
  - The bridge lets Gazebo motion, RViz visibility, and failure plumbing move forward now.
  - It keeps the eventual Nav2 swap isolated to the robot-local execution side.

### Graph-based path planning with BFS

Chosen because:

- The current environment is waypoint-based rather than continuous geometry-based navigation.
- BFS is simple, deterministic, easy to inspect, and sufficient for an unweighted demo graph.
- It is the smallest correct planning approach for the current V1 scope.

Compared with alternatives:

- Compared with Dijkstra or A*:
  - Those are better when edge costs or heuristics matter.
  - BFS is simpler and adequate while all graph edges are effectively equal-cost.
- Compared with Nav2 integration:
  - Nav2 would add significant system complexity too early.
  - The current goal is fleet coordination logic, not full navigation stack integration.

### Standalone workspace isolation

Chosen because:

- The project requirement explicitly says not to affect other work under `/home/bruce/project-a`.
- Isolation reduces accidental coupling and makes iteration safer.
- It simplifies build, status tracking, and Git history for this project.

Compared with alternatives:

- Compared with adding packages into an existing robotics workspace:
  - A shared workspace could create unrelated build and dependency risk.
- Compared with a non-isolated prototype directory:
  - The current structure is cleaner for long-term maintenance.

## Why This Approach Fits The Current Phase

This project is currently in a V1 simulation phase. The chosen stack favors:

- small, testable increments
- clear package boundaries
- ROS 2-native communication patterns
- low algorithmic complexity where that complexity is not yet needed

That is why the project currently uses:

- BFS instead of weighted or heuristic planning
- an external navigation bridge instead of direct Nav2 action integration in this environment
- distance-ranked assignment instead of optimization-heavy scheduling
- launch-time parameter configuration for graph topology and waypoint coordinates

## Current Tradeoffs

Benefits of the adopted approach:

- Fast to understand and extend
- Matches ROS 2 conventions
- Good separation between interfaces, dispatch, planning, and execution
- Keeps the codebase focused on coordination logic

Costs of the adopted approach:

- Demo map data is still simple ROS parameter YAML rather than a richer map format
- Scheduling is intentionally naive
- Direct Nav2 integration is blocked by missing development packages in the current environment
- Gazebo simulation still needs longer end-to-end runtime verification
- Sandbox limitations prevent full ROS 2 CLI runtime verification here

## Planned Evolution Path

The current approach is intended to evolve in this order:

1. Verify Gazebo bringup, robot spawning, and task-driven motion in a normal host shell
2. Replace the temporary goal-following bridge with direct Nav2 integration once Nav2 packages are installed
3. Add richer failure recovery and retry logic
4. Improve scheduling with ETA- and priority-aware behavior

## Documentation Policy

To keep documentation complete and useful:

- `BUGS.md` is the source of truth for all meaningful bugs and environmental blockers
- `STATUS.md` is the source of truth for verified behavior, limitations, and next steps
- `PROGRESS.md` is the chronological change log
- `TECHNICAL_APPROACH.md` is the source of truth for architecture and technology rationale
