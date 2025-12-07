<!--
---
sync_impact_report:
  version_change: "0.0.0 -> 1.0.0"
  modified_principles:
    - "PRINCIPLE_1_NAME -> Accuracy"
    - "PRINCIPLE_2_NAME -> Clarity"
    - "PRINCIPLE_3_NAME -> Reproducibility"
    - "PRINCIPLE_4_NAME -> Zero Hallucination"
    - "PRINCIPLE_5_NAME -> Tested Code"
  added_sections:
    - "Principle: Consistent Pedagogy"
    - "Principle: Diagrams"
    - "Principle: Grounded RAG"
    - "Deliverables"
    - "Domain Scope"
    - "Constraints"
  removed_sections: []
  templates_updated:
    - path: ".specify/templates/plan-template.md"
      status: "✅"
    - path: ".specify/templates/spec-template.md"
      status: "✅"
    - path: ".specify/templates/tasks-template.md"
      status: "✅"
  todos: []
---
-->
# Physical AI & Humanoid Robotics Constitution

## Core Principles

### Accuracy
Accuracy from primary/official robotics sources.

### Clarity
Clarity for senior CS/AI students.

### Reproducibility
Reproducible simulations and code.

### Zero Hallucination
Zero hallucination. All claims traceable and cited (APA).

### Tested Code
Tested code (ROS 2, Gazebo, Unity, Isaac).

### Consistent Pedagogy
Consistent pedagogy: Intro → Theory → Code → Example.

### Diagrams
Diagrams for complex systems.

### Grounded RAG
RAG chatbot answers only from book content or user-selected text.

## Deliverables

- Docusaurus book deployed to GitHub Pages
- Integrated RAG chatbot (OpenAI Agents/ChatKit, FastAPI, Neon Postgres, Qdrant)

## Domain Scope

Physical AI, humanoid robotics, ROS 2, URDF, Gazebo physics, Unity rendering, Isaac Sim/ROS, SLAM, Nav2, VLA, Whisper voice commands, cognitive planning, RAG systems.

## Constraints

- Accurate, runnable examples
- Zero plagiarism
- Chatbot grounded 100% in text
- Ubuntu 22.04, ROS 2 Humble/Iron, Gazebo, Isaac 4.x
- No unsafe robot-operation instructions

## Governance

All future `/sp.*` outputs must comply with this constitution. If a request conflicts, respond: “Request violates constitution.”

**Version**: 1.0.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07