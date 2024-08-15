from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, Condition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments for model names
    vlm_model_arg = DeclareLaunchArgument(
        'vlm_model',
        default_value='llava',
        description='VLM model name'
    )
    
    llm_model_arg = DeclareLaunchArgument(
        'llm_model',
        default_value='llama3.1',
        description='LLM model name'
    )

    # Declare launch argument for running webcam node
    run_webcam_node_arg = DeclareLaunchArgument(
        'run_webcam_node',
        default_value='true',
        description='Whether to run the webcam node'
    )

    # Create the ros2_llm_node
    ros2_llm_node = Node(
        package='ros2_llm',  # Replace with your actual package name
        executable='ros2_llm_node',
        name='ros2_llm_node',
        output='screen',
        parameters=[{
            'vlm_model': LaunchConfiguration('vlm_model'),
            'llm_model': LaunchConfiguration('llm_model')
        }]
    )

    # Create the webcam_vlm_node
    webcam_vlm_node = Node(
        package='ros2_llm',  # Replace with your actual package name
        executable='webcam_vlm_node',
        name='webcam_vlm_node',
        output='screen',
        condition=Condition(LaunchConfiguration('run_webcam_node'))
    )

    return LaunchDescription([
        vlm_model_arg,
        llm_model_arg,
        run_webcam_node_arg,
        ros2_llm_node,
        webcam_vlm_node
    ])