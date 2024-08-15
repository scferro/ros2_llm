from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments for model names
    vlm_model_arg = DeclareLaunchArgument(
        'vlm_model',
        default_value='llava-llama3',
        description='VLM model name'
    )
    
    llm_model_arg = DeclareLaunchArgument(
        'llm_model',
        default_value='llama3.1',
        description='LLM model name'
    )

    # Create the node
    ros2_llm_node = Node(
        package='your_package_name',  # Replace with your actual package name
        executable='ros2_llm_node',
        name='ros2_llm_node',
        output='screen',
        parameters=[{
            'vlm_model': LaunchConfiguration('vlm_model'),
            'llm_model': LaunchConfiguration('llm_model')
        }]
    )

    return LaunchDescription([
        vlm_model_arg,
        llm_model_arg,
        ros2_llm_node
    ])