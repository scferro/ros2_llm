import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ros2_llm_interfaces.srv import InferenceService
from ollama import generate
import cv2

class ROS2LLMNode(Node):
    def __init__(self):
        super().__init__('ros2_llm_node')
        
        # Declare parameters for model names
        self.declare_parameter('vlm_model', 'llava-llama3')
        self.declare_parameter('llm_model', 'llama3.1')
        
        # Get parameter values
        self.vlm_model = self.get_parameter('vlm_model').value
        self.llm_model = self.get_parameter('llm_model').value
        
        # Create services
        self.vlm_srv = self.create_service(InferenceService, 'query_vlm', self.query_vlm_callback)
        self.llm_srv = self.create_service(InferenceService, 'query_llm', self.query_llm_callback)
        
        self.bridge = CvBridge()
        self.get_logger().info('ROS2 LLM Node is ready.')
        self.get_logger().info(f'VLM Model: {self.vlm_model}')
        self.get_logger().info(f'LLM Model: {self.llm_model}')

    def query_vlm_callback(self, request, response):
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
        self.get_logger().info('Received an LLM request')
        
        prompt = request.prompt
        
        # Use the Ollama generate function with the configured LLM model
        full_response = ""
        for ollama_response in generate(self.llm_model, prompt, stream=True):
            full_response += ollama_response['response']
        
        response.response = full_response
        return response

def main(args=None):
    rclpy.init(args=args)
    ros2_llm_node = ROS2LLMNode()
    rclpy.spin(ros2_llm_node)
    ros2_llm_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()