# Research Plan: Physical AI & Humanoid Robotics Book

**Branch**: `001-ros2-humanoid-basics` | **Date**: 2025-12-07 | **Plan**: [specs/001-ros2-humanoid-basics/plan.md](specs/001-ros2-humanoid-basics/plan.md)

## Key Decisions Research

### 1. ROS 2 Humble/Iron Selection

**Decision**: ROS 2 Humble/Iron selection for code examples and reproducibility.
**Rationale for Research**: Ensure the chosen ROS 2 distribution aligns with long-term support, stability for the target audience (students), and compatibility with other tools (Gazebo, Isaac).
**Research Tasks**:
*   Compare EOL dates and community support for Humble vs Iron.
*   Check known compatibility issues with Gazebo and Isaac Sim for both distributions.
*   Evaluate ease of installation and setup for students on Ubuntu 22.04.

### 2. Gazebo vs Unity Roles

**Decision**: Define clear roles for Gazebo and Unity in the book.
**Rationale for Research**: Determine the best use cases for each simulator based on their strengths (e.g., Gazebo for physics-heavy, Unity for advanced rendering/user interaction).
**Research Tasks**:
*   Identify typical robotic scenarios where Gazebo excels.
*   Identify typical robotic scenarios where Unity (with ROS 2 integration) excels.
*   Evaluate available ROS 2 integration packages for Unity.

### 3. Isaac Sim Version

**Decision**: Select the specific version of Isaac Sim (e.g., 4.x) to use.
**Rationale for Research**: Isaac Sim is rapidly evolving. Ensure the chosen version provides the necessary features, stability, and is accessible to students.
**Research Tasks**:
*   Review release notes and feature sets of recent Isaac Sim 4.x versions.
*   Check system requirements and potential licensing implications for student use.
*   Identify stable ROS 2 integration capabilities for the chosen Isaac Sim version.

### 4. RAG Chunk Size + Embedding Model

**Decision**: Determine optimal RAG chunk size and embedding model for the chatbot.
**Rationale for Research**: These parameters significantly impact the accuracy and performance of the RAG system.
**Research Tasks**:
*   Investigate best practices for chunking strategies in document-based RAG systems.
*   Compare performance and cost of various open-source and commercial embedding models suitable for robotics/AI text.
*   Consider impact on Qdrant indexing and retrieval efficiency.

### 5. API Structure for Chatbot

**Decision**: Define the API structure for the integrated RAG chatbot.
**Rationale for Research**: Ensure the API is robust, scalable, and provides the necessary endpoints for ChatKit integration and future extensions.
**Research Tasks**:
*   Review common API design patterns for chatbot backends (RESTful, GraphQL considerations).
*   Define core endpoints for query submission, context management, and response retrieval.
*   Consider authentication/authorization mechanisms for the API.

## Tradeoffs Analysis Research

### 1. Simulation Realism vs Performance

**Tradeoff**: Balancing the need for realistic physics and visual fidelity in simulations against computational performance and student hardware requirements.
**Rationale for Research**: Students may have varying hardware capabilities; the book needs to provide runnable examples without requiring high-end machines.
**Research Tasks**:
*   Benchmark example URDFs and ROS 2 control loops in Gazebo and Unity for performance on mid-range hardware.
*   Explore options for simplifying simulation environments for educational purposes without losing key concepts.
*   Document minimum and recommended hardware specifications for running examples.

### 2. Code Depth vs Student Accessibility

**Tradeoff**: Deciding the level of detail and complexity in code examples to balance comprehensive learning with ease of understanding for students.
**Rationale for Research**: Too complex, and students might get lost; too simple, and they might not grasp advanced concepts.
**Research Tasks**:
*   Identify common pain points for students learning ROS 2 and robotics programming.
*   Review pedagogical approaches in existing robotics textbooks for code example structure and annotation.
*   Plan for supplementary materials (e.g., simpler versions, advanced challenges) where appropriate.

### 3. RAG Accuracy vs Latency

**Tradeoff**: Optimizing the RAG system for high accuracy in answering questions while maintaining low response latency for a good user experience.
**Rationale for Research**: A slow or inaccurate chatbot would detract from its utility as a learning tool.
**Research Tasks**:
*   Benchmark different Qdrant configurations and embedding models for retrieval speed.
*   Investigate caching strategies and distributed deployment options for FastAPI/Qdrant.
*   Evaluate methods for measuring and improving the grounding of RAG responses.
