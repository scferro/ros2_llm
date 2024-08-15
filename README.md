# ros2_llm README

## Overview

**ros2_llm** is a ROS 2 package that integrates Large Language Models (LLMs) and Vision-Language Models (VLMs) with ROS 2 systems. This package provides a framework for using local LLMs and VLMs within a ROS 2 environment through Ollama, enabling advanced natural language processing and AI-driven functionalities.

## Installation

Follow these steps to set up the **ros2_llm** package on your local machine:

1. **Clone the repository in the `/src/` directory of your workspace**:

    ```bash
    git clone https://github.com/scferro/ros2_llm.git
    ```

2. **Navigate into the repo**:

    ```bash
    cd ros2_llm
    ```

3. **Install dependencies**:

    Ensure you are in the root of your ROS 2 workspace (e.g., `~/ros2_ws`). Then run:

    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```

4. **Install Ollama**:

    Install Ollama by following the official installation guide: [Ollama Installation](https://ollama.com/download)

5. **Download and set up models with Ollama**:

    Download the LLaMA 3.1 and LLaVA-LLaMA3 models:

    ```bash
    ollama pull llama3
    ollama pull llava-llama3
    ```

6. **Install Ollama Python SDK**:

    Install the Ollama Python SDK to interact with the models programmatically:

    ```bash
    pip install ollama
    ```

7. **Build the package**:

    Use Colcon to build the package:

    ```bash
    colcon build --packages-select ros2_llm
    ```

8. **Source the setup file**:

    Source the setup file to overlay the newly built package into your ROS 2 environment:

    ```bash
    source install/setup.bash
    ```

    This command will start the node that integrates the LLM and VLM models with your ROS 2 system. The node will be configured to use the `llava-llama3` model for vision-language processing and `llama3.1` for text-based processing by default. 

    The node logs will confirm the models in use:
    
    ```
    [INFO] [ros2_llm_node]: ROS2 LLM Node is ready.
    [INFO] [ros2_llm_node]: VLM Model: llava-llama3
    [INFO] [ros2_llm_node]: LLM Model: llama3.1
    ```

## Using the Services

Once the package is running, you can interact with the LLM and VLM through the provided ROS 2 services:

1. **Call the Vision-Language Model (VLM) service**:

    You can send a prompt along with images to the VLM for processing using the following command:

    ```bash
    ros2 service call /query_vlm ros2_llm_interfaces/srv/InferenceService "{prompt: 'your prompt here', images: [your image data here]}"
    ```

    Replace `'your prompt here'` with the prompt you want the VLM to process, and include the relevant image data.

    Example command for sending a prompt without images:
    
    ```bash
    ros2 service call /query_vlm ros2_llm_interfaces/srv/InferenceService "{prompt: 'Describe the content of the image.'}"
    ```

2. **Call the Language Model (LLM) service**:

    You can send a text prompt to the LLM for processing using the following command:

    ```bash
    ros2 service call /query_llm ros2_llm_interfaces/srv/InferenceService "{prompt: 'your text here'}"
    ```

    Replace `'your text here'` with the text prompt you want the LLM to process.

    Example command:
    
    ```bash
    ros2 service call /query_llm ros2_llm_interfaces/srv/InferenceService "{prompt: 'What is the capital of France?'}"
    ```

3. **Check service response**:

    The response from the LLM or VLM will be displayed in the terminal, showing the model's output based on the provided input.

## Customization

You can customize the LLM and VLM behavior by modifying the model parameters in the ROS 2 node. The parameters `vlm_model` and `llm_model` can be set to different model names supported by Ollama.

To change the models, update the parameters when launching the node:

```bash
ros2 run ros2_llm ros2_llm_node --ros-args -p vlm_model:=your_vlm_model -p llm_model:=your_llm_model

## Contributing

Contributions to this project are welcome! If you encounter any issues or have suggestions for improvements, please open an issue or submit a pull request.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.

---
