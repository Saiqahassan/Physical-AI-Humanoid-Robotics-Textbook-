# Tasks: ROS 2 Humanoid Basics Module

**Input**: Design documents from `/specs/001-ros2-humanoid-basics/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root
- **Web app**: `backend/src/`, `frontend/src/`
- **Mobile**: `api/src/`, `ios/src/` or `android/src/`
- Paths shown below assume single project - adjust based on plan.md structure

## Guiding Principles

- **Tested Code**: Remember to include tasks for testing all code (ROS 2, Gazebo, Unity, Isaac).
- **Consistent Pedagogy**: Structure tasks to follow the "Intro → Theory → Code → Example" pattern where applicable.

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create the main `book-project/` directory.
- [x] T002 Create `docusaurus-site/`, `rag-backend/`, `robotics-code/`, `tools/` within `book-project/`.
- [x] T003 Initialize Docusaurus project within `book-project/docusaurus-site/`.
- [x] T004 Create initial Docusaurus `docusaurus.config.js` and `src/components/` structure in `book-project/docusaurus-site/`.
- [x] T005 Initialize FastAPI project within `book-project/rag-backend/`.
- [x] T006 Create initial FastAPI directory structure: `rag-backend/app/api/`, `rag-backend/app/services/`, `rag-backend/app/core/`.
- [x] T007 Initialize `robotics-code/` with subdirectories `ros2-examples/`, `urdf-models/`, `gazebo-sims/`, `unity-sims/`, `isaac-sims/`.
- [x] T008 Create `tools/` directory for miscellaneous scripts.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T009 Configure Git to ignore generated files and large binaries within `book-project/docusaurus-site/`, `book-project/rag-backend/`, `book-project/robotics-code/`.
- [x] T010 Setup Docusaurus documentation structure for `module1-ros2/` within `book-project/docusaurus-site/docs/`.
- [x] T011 [P] Setup basic Qdrant client configuration in `book-project/rag-backend/app/core/vector_db.py`.
- [x] T012 [P] Setup basic Neon (PostgreSQL) client configuration in `book-project/rag-backend/app/core/db.py`.
- [x] T013 [P] Implement basic health check endpoint for FastAPI in `book-project/rag-backend/app/api/health.py` based on `contracts/chatbot-api.yaml`.
- [x] T014 Create base RAG service structure in `book-project/rag-backend/app/services/rag_service.py`.

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Setup and Minimal ROS 2 Node Graph (Priority: P1) 🎯 MVP

**Goal**: A student can successfully set up their environment and run a minimal ROS 2 node graph.

**Independent Test**: Successfully execute minimal ROS 2 talker/listener nodes on Ubuntu 22.04 with ROS 2 Humble/Iron, demonstrating basic communication.

### Implementation for User Story 1

- [x] T015 [US1] Write content for "ROS 2 as the Robotic Nervous System" chapter in `book-project/docusaurus-site/docs/module1-ros2/01-ros2-nervous-system.md`.
- [x] T016 [US1] Create a minimal ROS 2 Python publisher (`talker.py`) in `book-project/robotics-code/ros2-examples/minimal_nodes/talker.py`.
- [x] T017 [US1] Create a minimal ROS 2 Python subscriber (`listener.py`) in `book-project/robotics-code/ros2-examples/minimal_nodes/listener.py`.
- [x] T018 [US1] Add ROS 2 package configuration (`setup.py`, `package.xml`, `resource/`) for `minimal_nodes` in `book-project/robotics-code/ros2-examples/minimal_nodes/`.
- [x] T019 [US1] Integrate `minimal_nodes` example into Docusaurus documentation, including setup instructions and expected output in `book-project/docusaurus-site/docs/module1-ros2/01-ros2-nervous-system.md`.
- [x] T020 [P] [US1] Create diagram for ROS 2 node graph in `book-project/docusaurus-site/static/assets/ros2_node_graph.png`.

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Publish/Subscribe with rclpy (Priority: P1)

**Goal**: A student can understand how ROS 2 nodes communicate using topics and implement publish/subscribe logic with `rclpy`.

**Independent Test**: A student can write and run simple Python nodes that publish and subscribe to a custom topic, verifying message exchange and understanding the `rclpy` API.

### Implementation for User Story 2

- [x] T021 [US2] Write content for "Python Agents + ROS 2 (rclpy)" chapter in `book-project/docusaurus-site/docs/module1-ros2/02-python-rclpy.md`.
- [x] T022 [US2] Create an `rclpy` publisher example (`simple_publisher.py`) in `book-project/robotics-code/ros2-examples/pubsub_rclpy/simple_publisher.py`.
- [x] T023 [US2] Create an `rclpy` subscriber example (`simple_subscriber.py`) in `book-project/robotics-code/ros2-examples/pubsub_rclpy/simple_subscriber.py`.
- [x] T024 [US2] Add ROS 2 package configuration for `pubsub_rclpy` in `book-project/robotics-code/ros2-examples/pubsub_rclpy/`.
- [x] T025 [US2] Integrate `pubsub_rclpy` examples into Docusaurus documentation, explaining concepts and code walkthrough in `book-project/docusaurus-site/docs/module1-ros2/02-python-rclpy.md`.

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Building a Basic Humanoid URDF in RViz/Gazebo (Priority: P1)

**Goal**: A student can learn how to describe a robot with URDF and visualize it in RViz/Gazebo.

**Independent Test**: A simple humanoid URDF model is created, loads correctly, and is displayed accurately in both RViz and Gazebo.

### Implementation for User Story 3

- [x] T026 [US3] Write content for "Building a Basic Humanoid URDF" chapter in `book-project/docusaurus-site/docs/module1-ros2/03-humanoid-urdf.md`.
- [x] T027 [US3] Create a basic humanoid URDF XML file in `book-project/robotics-code/urdf-models/simple_humanoid.urdf`.
- [x] T028 [US3] Create an RViz configuration file for visualizing the URDF in `book-project/robotics-code/urdf-models/simple_humanoid.rviz`.
- [x] T029 [US3] Develop a ROS 2 launch file (`display_humanoid.launch.py`) to load the URDF in RViz and Gazebo in `book-project/robotics-code/ros2-examples/urdf_display/`.
- [x] T030 [US3] Integrate URDF and visualization instructions into Docusaurus documentation in `book-project/docusaurus-site/docs/module1-ros2/03-humanoid-urdf.md`.
- [x] T031 [US3] Add ROS 2 package configuration for `urdf_display` in `book-project/robotics-code/ros2-examples/urdf_display/`.

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and overall system quality.

- [x] T032 Review all Module 1 content (`book-project/docusaurus-site/docs/module1-ros2/`) for consistency, clarity, and pedagogical approach.
- [x] T033 Verify all code examples in `book-project/robotics-code/ros2-examples/` are runnable and produce expected output on the specified environment.
- [x] T034 Ensure all diagrams in `book-project/docusaurus-site/static/assets/` are correctly referenced and displayed in Docusaurus content.
- [x] T035 Perform Docusaurus build to check for errors and broken links within `book-project/docusaurus-site/`.
- [x] T036 Set up automated testing for `rag-backend/` components using `pytest` in `book-project/rag-backend/tests/`.
- [x] T037 Implement the `/chat` endpoint logic in `book-project/rag-backend/app/api/chat.py`, integrating with the RAG service and Qdrant/Neon.
- [x] T038 Develop an initial RAG content ingestion script in `book-project/tools/ingest_rag_content.py` for Module 1 content.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P1)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

### Within Each User Story

- Content creation before example integration.
- Code examples developed before being integrated into documentation.
- Models (e.g., URDF) developed before visualization instructions.
- Story complete before moving to next priority.

### Parallel Opportunities

- All tasks marked [P] can run in parallel (T011, T012, T013 for Foundational; T020 for US1).
- Once Foundational phase completes, all user stories (Phase 3, 4, 5) can start in parallel (if team capacity allows).
- Within each user story, tasks related to content writing and code example development can often be parallelized (e.g., T015, T016, T017 for US1 if different people work on content vs code).

---

## Parallel Example: User Story 1

```bash
# Writing content and creating diagrams can be parallelized:
Task: "T015 [US1] Write content for 'ROS 2 as the Robotic Nervous System' chapter in book-project/docusaurus-site/docs/module1-ros2/01-ros2-nervous-system.md"
Task: "T020 [P] [US1] Create diagram for ROS 2 node graph in book-project/docusaurus-site/static/assets/ros2_node_graph.png"

# Creating code examples can be parallelized:
Task: "T016 [US1] Create a minimal ROS 2 Python publisher (talker.py) in book-project/robotics-code/ros2-examples/minimal_nodes/talker.py"
Task: "T017 [US1] Create a minimal ROS 2 Python subscriber (listener.py) in book-project/robotics-code/ros2-examples/minimal_nodes/listener.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
