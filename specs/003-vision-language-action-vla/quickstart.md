# Quickstart: Vision-Language-Action Module

This guide provides the essential steps to get the VLA module simulation up and running.

## 1. Prerequisites

- **Operating System**: Ubuntu 22.04
- **ROS 2**: ROS 2 Humble or Iron, with Colcon and other development tools installed.
- **Gazebo**: Gazebo Fortress installed and integrated with ROS 2.
- **OpenAI API Key**: You need an API key from OpenAI with access to the Whisper and GPT-4 models.

## 2. Installation

1.  **Clone the repository**:
    ```bash
    git clone https://github.com/your-username/your-repo.git
    cd your-repo
    ```

2.  **Set up your OpenAI API Key**:
    Export your API key as an environment variable. It's recommended to add this to your `~/.bashrc` file.
    ```bash
    export OPENAI_API_KEY='your-api-key-here'
    ```

3.  **Install ROS 2 Dependencies**:
    From the root of the workspace, install any dependencies listed in the `package.xml` files.
    ```bash
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
    ```

4.  **Install Python Dependencies**:
    The Python-based ROS 2 nodes will have dependencies listed in a `requirements.txt` file.
    ```bash
    pip install -r src/vla_module/vla_core/requirements.txt
    ```

5.  **Build the ROS 2 Workspace**:
    ```bash
    colcon build
    ```

## 3. Running the Simulation

1.  **Source the workspace**:
    ```bash
    source install/setup.bash
    ```

2.  **Launch the Gazebo simulation**:
    This will launch Gazebo with the humanoid robot in the specified world.
    ```bash
    ros2 launch vla_gazebo simulation.launch.py
    ```

## 4. Running the VLA Pipeline

1.  **Source the workspace** in a new terminal.

2.  **Launch the VLA core nodes**:
    This will start the speech-to-text node, the LLM planner, and the robot controller.
    ```bash
    ros2 launch vla_core vla_pipeline.launch.py
    ```

3.  **Start speaking commands**:
    With the pipeline running, you can now issue spoken commands to the robot. The robot should respond and act in the Gazebo simulation.
