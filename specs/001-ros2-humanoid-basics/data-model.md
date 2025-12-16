# Data Model: Physical AI & Humanoid Robotics Book Content

**Branch**: `001-ros2-humanoid-basics` | **Date**: 2025-12-07 | **Plan**: [specs/001-ros2-humanoid-basics/plan.md](specs/001-ros2-humanoid-basics/plan.md)

This data model describes the logical entities that structure the book's content and the components of the integrated RAG (Retrieval-Augmented Generation) chatbot system. While this is not a traditional software data model with database tab


## Logical Entities

### 1. BookModule

Represents a major section or module of the book (e.g., "Module 1: ROS 2").

*   **Attributes**:
    *   `Title`: String (e.g., "Module 1: The Robotic Nervous System (ROS 2)").
    *   `Slug`: String (e.g., "module1-ros2"). Unique identifier for URL/path.
    *   `Description`: String (Brief overview of the module's content).
    *   `Chapters`: List of `Chapter` entities (Subsections within this module).
    *   `CodeExamples`: List of `CodeExample` entities (Associated code snippets/projects).
    *   `Diagrams`: List of `Diagram` entities (Associated diagrams/illustrations).
    *   `LearningObjectives`: List of Strings (Key takeaways for the student).
*   **Relationships**: Contains `Chapter`, `CodeExample`, and `Diagram` entities.

### 2. Chapter

Represents a specific sub-section or topic within a `BookModule`.

*   **Attributes**:
    *   `Title`: String (e.g., "ROS 2 as the Robotic Nervous System").
    *   `Slug`: String (e.g., "ros2-nervous-system"). Unique identifier.
    *   `Content`: Markdown text (The primary instructional content of the chapter).
    *   `Order`: Integer (Sequence within its `BookModule`).
*   **Relationships**: Belongs to a `BookModule`.

### 3. CodeExample

Represents a runnable code snippet, script, or project provided as an example.

*   **Attributes**:
    *   `Name`: String (e.g., "ROS 2 Talker-Listener Example").
    *   `Description`: String (Explanation of what the code does).
    *   `Language`: String (e.g., "Python", "URDF XML", "C++").
    *   `FilePath`: String (Relative path to the code file in `robotics-code/`).
    *   `Dependencies`: List of Strings (e.g., "rclpy", "ament_cmake", "gazebo_ros_pkgs").
    *   `ExpectedOutput`: String (Description of what should happen when the code runs).
*   **Relationships**: Associated with one or more `Chapter` entities.

### 4. Diagram

Represents a diagram, image, or illustration used within the book.

*   **Attributes**:
    *   `Name`: String (e.g., "ROS 2 Node Graph Diagram").
    *   `Description`: String (Brief explanation of the diagram's content).
    *   `FilePath`: String (Relative path to the image file in `docusaurus-site/static/assets/`).
    *   `AltText`: String (Alternative text for accessibility).
*   **Relationships**: Associated with one or more `Chapter` entities.

### 5. RAGDocument

Represents a semantically coherent chunk of text extracted from the book content, used for indexing in the RAG system.

*   **Attributes**:
    *   `Content`: String (The actual text chunk).
    *   `SourceModule`: String (Title of the `BookModule` it came from).
    *   `SourceChapter`: String (Title of the `Chapter` it came from).
    *   `SourceFilePath`: String (Path to the original Markdown file).
    *   `PageNumber`: Integer (Approximate page number or section index).
    *   `EmbeddingVector`: Vector of Floats (Numerical representation of the `Content`).
    *   `Metadata`: JSON/Dictionary (Additional contextual information for retrieval).
*   **Relationships**: Stored in Qdrant; derived from `Chapter` content.

### 6. Query

Represents a user's input question to the RAG chatbot.

*   **Attributes**:
    *   `Text`: String (The user's question).
    *   `Timestamp`: Datetime.
    *   `SessionID`: String (Identifier for a continuous conversation).
    *   `UserID`: String (Optional, for personalization/logging).
*   **Relationships**: Leads to one or more `Response` entities.

### 7. Response

Represents the RAG chatbot's generated answer to a `Query`.

*   **Attributes**:
    *   `Text`: String (The generated answer).
    *   `Timestamp`: Datetime.
    *   `RetrievedDocuments`: List of `RAGDocument` (The chunks used to generate the answer).
    *   `ConfidenceScore`: Float (Optional, how certain the chatbot is of its answer).
    *   `Sources`: List of Strings (Citations/links back to the book content).
*   **Relationships**: Generated in response to a `Query`.
