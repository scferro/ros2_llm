from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
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

    # Declare launch arguments for webcam node
    camera_index_arg = DeclareLaunchArgument(
        'camera_index',
        default_value='0',
        description='Camera index for webcam'
    )

    frame_rate_arg = DeclareLaunchArgument(
        'frame_rate',
        default_value='30.0',
        description='Frame rate for webcam capture'
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
        parameters=[{
            'camera_index': LaunchConfiguration('camera_index'),
            'frame_rate': LaunchConfiguration('frame_rate'),
        }]
    )

    return LaunchDescription([
        vlm_model_arg,
        llm_model_arg,
        camera_index_arg,
        frame_rate_arg,
        ros2_llm_node,
        webcam_vlm_node
    ])