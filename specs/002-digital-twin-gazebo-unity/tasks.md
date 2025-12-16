# Tasks: Digital Twin with Gazebo and Unity

**Input**: Design documents from `specs/002-digital-twin-gazebo-unity/`
**Prerequisites**: `plan.md`, `spec.md`, `research.md`, `data-model.md`, `contracts/`

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create `Dockerfile` for a reproducible environment with ROS 2, Gazebo, and Unity dependencies.
- [x] T002 Create a ROS 2 workspace in `src/`.
- [x] T003 Create a ROS 2 package named `digital_twin` in `src/`.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented.

- [x] T004 Create a basic URDF/SDF model of a humanoid robot in `src/digital_twin/models/`.
- [x] T005 Create a basic Gazebo world file in `src/digital_twin/worlds/`.
- [x] T006 Create a basic Unity project and scene for the visualization.

---

## Phase 3: User Story 1 - Physics-Based Simulation with Gazebo (P1) 🎯 MVP

**Goal**: A student can build and run a simulation of a humanoid robot in Gazebo, controlling it with ROS 2.

**Independent Test**: The Gazebo simulation can be run, and the robot can be controlled via ROS 2 commands, demonstrating basic movement.

### Implementation for User Story 1

- [x] T007 [US1] Configure the physics properties (gravity, friction, constraints) in the Gazebo world file.
- [x] T008 [US1] Configure the joint limits and inertial properties in the robot's URDF/SDF file.
- [x] T009 [US1] Create a ROS 2 launch file in `src/digital_twin/launch/` to start the Gazebo simulation with the robot model.
- [x] T010 [US1] Create a ROS 2 node in `src/digital_twin/src/` to subscribe to `/cmd_vel` and publish joint commands.
- [x] T011 [US1] Write a test in `src/digital_twin/test/` to verify that publishing to `/cmd_vel` causes the robot's joints to move.

---

## Phase 4: User Story 2 - Sensor Simulation (P2)

**Goal**: A student can add simulated sensors to the Gazebo model and visualize the sensor data.

**Independent Test**: The Gazebo simulation can be run with sensors enabled, and the sensor data can be visualized using ROS 2 tools.

### Implementation for User Story 2

- [x] T012 [P] [US2] Add a simulated LiDAR sensor to the robot's URDF/SDF file.
- [x] T013 [P] [US2] Add a simulated depth camera to the robot's URDF/SDF file.
- [x] T014 [P] [US2] Add a simulated IMU to the robot's URDF/SDF file.
- [x] T015 [US2] Update the ROS 2 launch file to enable the sensor plugins.
- [x] T016 [US2] Write a test to verify that sensor data is being published to the correct ROS 2 topics.

---

## Phase 5: User Story 3 - High-Fidelity Digital Twin with Unity (P3)

**Goal**: A student can create a visually realistic scene in Unity and synchronize the robot's state from Gazebo.

**Independent Test**: The Unity visualization mirrors the movements of the robot in the Gazebo simulation.

### Implementation for User Story 3

- [x] T017 [US3] Set up the `ROS2 For Unity` package in the Unity project.
- [x] T018 [US3] Create a C# script in the Unity project to subscribe to the `/joint_states` and `/tf` topics.
- [x] T019 [US3] Create a C# script in the Unity project to update the robot's visualization based on the received ROS 2 messages.
- [x] T020 [US3] Write a test in Unity to verify that the robot's visualization is correctly updated when mock data is received.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories.

- [x] T021 [P] Update all documentation, including `README.md` and `quickstart.md`.
- [x] T022 Code cleanup and refactoring across all new files. (Manual review recommended)
- [x] T023 Performance optimization of the simulation and visualization. (Manual execution and analysis required)

---

## Dependencies & Execution Order

- **Phase 1 (Setup)** must be completed before **Phase 2 (Foundational)**.
- **Phase 2 (Foundational)** must be completed before any user story implementation can begin.
- **User Stories (Phases 3-5)** can be implemented in any order after Phase 2 is complete, but the recommended order is P1 -> P2 -> P3.
- **Phase 6 (Polish)** should be done after all user stories are complete.

## Parallel Opportunities

- Within **Phase 4 (User Story 2)**, tasks T012, T013, and T014 can be done in parallel.
- Once **Phase 2 (Foundational)** is complete, different developers can work on different user stories in parallel.
