# Tasks: Module 4: Vision-Language-Action (VLA)

**Input**: Design documents from `specs/003-vision-language-action-vla/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create ROS 2 packages `vla_description`, `vla_gazebo`, and `vla_core` in `src/vla_module/`.
- [X] T002 Initialize Python packages with `ament_python` and set up `package.xml` and `setup.py` for each package.
- [X] T003 Create documentation structure in `book-project/docusaurus-site/docs/module4/`.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented.

- [X] T004 Create the URDF for the humanoid robot in `src/vla_module/vla_description/urdf/humanoid.urdf`.
- [X] T005 Create a basic Gazebo world in `src/vla_module/vla_gazebo/worlds/vla_world.world`.
- [X] T006 Create a launch file to spawn the robot in the Gazebo world in `src/vla_module/vla_gazebo/launch/simulation.launch.py`.

---

## Phase 3: User Story 1 - Voice-to-Action Pipeline (Priority: P1) 🎯 MVP

**Goal**: Implement a basic pipeline that takes a spoken command, transcribes it, and makes the robot perform a simple, corresponding action in the simulation.

**Independent Test**: Speak a command like "Wave hello" and verify that the robot performs the waving action in Gazebo.

### Implementation for User Story 1

- [X] T007 [US1] Implement the `speech_to_text_node.py` in `src/vla_module/vla_core/nodes/` to capture audio and use the Whisper API.
- [X] T008 [US1] Implement the `robot_controller_node.py` in `src/vla_module/vla_core/nodes/` with basic, hard-coded actions (e.g., wave, walk forward).
- [X] T009 [US1] Create a launch file `src/vla_module/vla_core/launch/voice_control.launch.py` to run the speech-to-text and controller nodes.
- [X] T010 [US1] Write the first chapter of the book, `1-voice-to-action.md`, in `book-project/docusaurus-site/docs/module4/`.

---

## Phase 4: User Story 2 - LLM-based Task Planning (Priority: P2)

**Goal**: Use an LLM to decompose high-level text commands into a sequence of actions for the robot.

**Independent Test**: Provide a text command like "get the ball" and verify that the LLM planner node outputs a logical sequence of actions (e.g., navigate, find, pick up).

### Implementation for User Story 2

- [X] T011 [US2] Implement the `llm_planner_node.py` in `src/vla_module/vla_core/nodes/` to take text commands and generate structured action plans using the GPT-4 API.
- [X] T012 [US2] Modify `robot_controller_node.py` to subscribe to action plans and execute them sequentially.
- [X] T013 [US2] Update the launch file `src/vla_module/vla_core/launch/voice_control.launch.py` to include the `llm_planner_node`.
- [X] T014 [US2] Write the second chapter, `2-cognitive-planning.md`, in `book-project/docusaurus-site/docs/module4/`.

---

## Phase 5: User Story 3 - Capstone: Autonomous Task Execution (Priority: P3)

**Goal**: Integrate visual perception to create a fully autonomous system that can execute a spoken command from end to end.

**Independent Test**: In the simulation, say "pick up the block". The robot should navigate to the block, see it, and pick it up.

### Implementation for User Story 3

- [X] T015 [US3] Implement a basic `perception_node.py` in `src/vla_module/vla_core/nodes/` that uses the robot's camera to detect the location of objects in Gazebo.
- [X] T016 [US3] Integrate the perception node with the `llm_planner_node.py` so the planner can incorporate real-time object locations into its plans.
- [X] T017 [US3] Extend `robot_controller_node.py` with a manipulation action (e.g., a "pick up" action server) that can interact with objects in the simulation.
- [X] T018 [US3] Write the third chapter, `3-capstone-project.md`, in `book-project/docusaurus-site/docs/module4/`.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories.

- [X] T019 Add basic tests for the ROS 2 nodes in `src/vla_module/vla_core/test/`.
- [X] T020 Review and refine all documentation in `book-project/docusaurus-site/docs/module4/`.
- [X] T021 Create the architectural diagrams as defined in `research.md` and embed them in the documentation.

---

## Dependencies & Execution Order

- **User Story 1 (P1)** is the foundation.
- **User Story 2 (P2)** depends on User Story 1.
- **User Story 3 (P3)** depends on User Story 2.
- The project should be implemented sequentially from US1 to US3.

## Implementation Strategy

The project will be delivered incrementally, following the user story priorities. The MVP will be the completion of User Story 1.
