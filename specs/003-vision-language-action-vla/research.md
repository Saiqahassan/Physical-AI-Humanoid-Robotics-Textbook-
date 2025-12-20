# Research: Module 4: Vision-Language-Action (VLA)

This document outlines the research tasks required to resolve ambiguities in the implementation plan.

## 1. Large Language Model (LLM) Selection

**Task**: Research and select a specific Large Language Model (LLM) suitable for cognitive planning in a robotics context, considering factors like performance, accessibility, and cost.

**Decision**: We will use **GPT-4** from OpenAI.

**Rationale**: 
- **State-of-the-Art Performance**: GPT-4 has demonstrated strong capabilities in reasoning, planning, and instruction following, which are crucial for decomposing high-level commands into actionable steps for a robot.
- **Accessibility**: The OpenAI API is well-documented and widely used, making it relatively easy to integrate into the ROS 2 ecosystem.
- **Existing Research**: There is a growing body of research and community projects that use GPT-series models for robotics tasks, providing a solid foundation and examples to draw from.

**Alternatives considered**:
- **Open-source models (e.g., Llama 3, Mistral)**: While powerful, they may require more effort to set up and fine-tune for this specific robotics application. For the purpose of a clear, reproducible book chapter, a well-established API is preferable.
- **Google's Gemini**: A strong contender, but GPT-4 currently has more public-facing examples and community support in the robotics space.

## 2. Key Architectural Diagrams

**Task**: Identify and outline the key diagrams required to explain the VLA architecture, including the data flow between the speech, perception, planning, and action components.

**Decision**: The following diagrams will be created:

1.  **High-Level VLA Architecture Diagram**: A block diagram showing the main components (Speech Input, Speech-to-Text, LLM Planner, Perception, Robot Controller, Simulation Environment) and the primary data flow between them.
2.  **Voice-to-Action Data Flow**: A sequence diagram illustrating the process from a spoken command to a robot action. It will show the interactions between the user, the microphone node, the Whisper transcription service, the LLM planner, and the ROS 2 action server.
3.  **ROS 2 Node Graph**: A diagram showing the ROS 2 nodes involved in the capstone project and the topics/services/actions they use to communicate.

**Rationale**: These diagrams will provide a clear visual representation of the system's architecture and data flow, making it easier for students to understand the complex interactions between the different components. This aligns with the **Clarity** and **Consistent Pedagogy** principles of the constitution.

**Alternatives considered**:
- **UML Component Diagrams**: While more formal, they can be less intuitive for a book chapter aimed at a broad student audience. Simple block and sequence diagrams are more effective for pedagogical purposes.
- **C4 Model**: Overkill for a single book chapter. A few well-chosen diagrams are sufficient to convey the architecture.
