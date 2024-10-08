"""
ROS2 LLM Node for integrating Large Language Models and Vision Language Models.

This node provides services for querying Vision Language Models (VLM) and
Large Language Models (LLM), as well as maintaining a chat session with an LLM.

SERVICES:
    + /query_vlm (InferenceService) - Query the Vision Language Model with images and text.
    + /query_llm (InferenceService) - Query the Language Model with text.
    + /chat_llm (InferenceService) - Interact with the Language Model in a chat session.
    + /reset_chat (InferenceService) - Reset the chat history.

PARAMETERS:
    + vlm_model (string) - Name of the Vision Language Model to use.
    + llm_model (string) - Name of the Language Model to use.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ros2_llm_interfaces.srv import InferenceService
from ollama import generate, chat
import cv2

class ROS2LLMNode(Node):
    def __init__(self):
        """Initialize the ROS2 LLM Node."""
        super().__init__('ros2_llm_node')
        
        # Declare parameters for model names
        self.declare_parameter('vlm_model', 'llava')
        self.declare_parameter('llm_model', 'llama3.1')
        
        # Get parameter values
        self.vlm_model = self.get_parameter('vlm_model').value
        self.llm_model = self.get_parameter('llm_model').value
        
        # Create services
        self.vlm_srv = self.create_service(InferenceService, 'query_vlm', self.query_vlm_callback)
        self.llm_srv = self.create_service(InferenceService, 'query_llm', self.query_llm_callback)
        self.chat_srv = self.create_service(InferenceService, 'chat_llm', self.chat_llm_callback)
        self.reset_srv = self.create_service(InferenceService, 'reset_chat', self.reset_chat_callback)
        
        self.bridge = CvBridge()
        self.chat_messages = []
        
        self.get_logger().info('ROS2 LLM Node is ready.')
        self.get_logger().info(f'VLM Model: {self.vlm_model}')
        self.get_logger().info(f'LLM Model: {self.llm_model}')

    def query_vlm_callback(self, request, response):
        """
        Service callback for querying the Vision Language Model.

        Args:
            request (InferenceService.Request): The service request containing prompt and images.
            response (InferenceService.Response): The service response to be filled.

        Returns:
            InferenceService.Response: The response containing the VLM's output.
        """
        self.get_logger().info('Received a VLM request')
        
        prompt = request.prompt
        images = request.images
        
        # Convert ROS Image messages to byte arrays
        image_data_list = []
        for img in images:
            cv_image = self.bridge.imgmsg_to_cv2(img, desired_encoding='passthrough')
            _, img_encoded = cv2.imencode('.jpg', cv_image)
            image_data = img_encoded.tobytes()
            image_data_list.append(image_data)

        # Use the Ollama generate function with the configured VLM model
        full_response = ""
        for ollama_response in generate(self.vlm_model, prompt, images=image_data_list, stream=True):
            full_response += ollama_response['response']

        response.response = full_response
        return response

    def query_llm_callback(self, request, response):
        """
        Service callback for querying the Language Model.

        Args:
            request (InferenceService.Request): The service request containing the prompt.
            response (InferenceService.Response): The service response to be filled.

        Returns:
            InferenceService.Response: The response containing the LLM's output.
        """
        self.get_logger().info('Received an LLM request')
        
        prompt = request.prompt
        
        # Use the Ollama generate function with the configured LLM model
        full_response = ""
        for ollama_response in generate(self.llm_model, prompt, stream=True):
            full_response += ollama_response['response']
        
        response.response = full_response
        return response

    def chat_llm_callback(self, request, response):
        """
        Service callback for chatting with the Language Model.

        Args:
            request (InferenceService.Request): The service request containing the user's message.
            response (InferenceService.Response): The service response to be filled.

        Returns:
            InferenceService.Response: The response containing the LLM's reply.
        """
        self.get_logger().info('Received a chat LLM request')
        
        user_message = request.prompt
        self.chat_messages.append({'role': 'user', 'content': user_message})
        
        # Use the Ollama chat function with the configured LLM model
        chat_response = chat(self.llm_model, messages=self.chat_messages)
        ai_message = chat_response['message']['content']
        
        self.chat_messages.append({'role': 'assistant', 'content': ai_message})
        
        response.response = ai_message
        return response

    def reset_chat_callback(self, request, response):
        """
        Service callback for resetting the chat history.

        Args:
            request (InferenceService.Request): The service request (unused).
            response (InferenceService.Response): The service response to be filled.

        Returns:
            InferenceService.Response: The response confirming the chat reset.
        """
        self.get_logger().info('Received a reset chat request')
        self.chat_messages = []
        response.response = "Chat history has been reset."
        return response

def main(args=None):
    """
    Main function to initialize and run the ROS2 LLM Node.

    Args:
        args: Command-line arguments (if any).
    """
    rclpy.init(args=args)
    ros2_llm_node = ROS2LLMNode()
    rclpy.spin(ros2_llm_node)
    ros2_llm_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()