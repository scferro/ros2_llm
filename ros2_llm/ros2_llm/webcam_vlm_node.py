"""
WebcamVLMNode: A ROS2 node for capturing webcam frames and processing them with a Vision Language Model (VLM).

This node captures frames from a webcam, publishes them as ROS2 Image messages,
and provides a service to describe the current webcam frame using a VLM.

PUBLISHERS:
    + /webcam (sensor_msgs/Image) - Publishes webcam frames.

SUBSCRIBERS:
    + /describe_webcam (std_msgs/Empty) - Triggers VLM inference on the current webcam frame.

SERVICE CLIENTS:
    + query_vlm (InferenceService) - Client for the VLM service.

PARAMETERS:
    + camera_index (int) - Index of the camera to use (default: 0).
    + frame_rate (float) - Frame rate for publishing webcam images (default: 30.0).
    + vlm_prompt (string) - Prompt to use for the VLM (default: 'Describe what you see in this image.').
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ros2_llm_interfaces.srv import InferenceService
from std_msgs.msg import Empty
import cv2

class WebcamVLMNode(Node):
    def __init__(self):
        """Initialize the WebcamVLMNode."""
        super().__init__('webcam_vlm_node')
        
        # Declare parameters
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('vlm_prompt', 'Describe what you see in this image.')
        
        # Get parameter values
        self.camera_index = self.get_parameter('camera_index').value
        self.frame_rate = self.get_parameter('frame_rate').value
        self.vlm_prompt = self.get_parameter('vlm_prompt').value
        
        # Initialize webcam
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open webcam')
            return
        
        # Create publisher for webcam images
        self.webcam_pub = self.create_publisher(Image, '/webcam', 10)
        
        # Create service client for VLM service
        self.vlm_client = self.create_client(InferenceService, 'query_vlm')
        
        # Create subscription for triggering VLM inference
        self.describe_webcam_sub = self.create_subscription(
            Empty,
            '/describe_webcam',
            self.describe_webcam_callback,
            10
        )
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Create timer for publishing webcam frames
        self.create_timer(1.0 / self.frame_rate, self.publish_webcam_frame)
        
        self.get_logger().info('Webcam VLM Node is ready.')
    
    def publish_webcam_frame(self):
        """Capture and publish a frame from the webcam."""
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.webcam_pub.publish(msg)
    
    def describe_webcam_callback(self, msg):
        """
        Callback function for the /describe_webcam topic.
        
        This function captures the current webcam frame and sends it to the VLM service for description.

        Args:
            msg (Empty): An empty message triggering the callback.
        """
        self.get_logger().info('Received a describe webcam request')
        
        # Capture the latest frame
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture frame')
            return
        
        self.get_logger().info('Frame captured successfully')
        
        # Convert frame to ROS Image message
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        
        self.get_logger().info('Frame converted to ROS Image message')
        
        # Prepare the request for the VLM service
        vlm_request = InferenceService.Request()
        vlm_request.prompt = self.vlm_prompt
        vlm_request.images = [img_msg]
        
        self.get_logger().info('VLM request prepared')
        
        # Call the VLM service
        if not self.vlm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('VLM service not available')
            return
        
        self.get_logger().info('VLM service is available, calling now')
        
        future = self.vlm_client.call_async(vlm_request)
        future.add_done_callback(self.vlm_response_callback)
    
    def vlm_response_callback(self, future):
        """
        Callback function for handling the VLM service response.

        Args:
            future (rclpy.Future): The future object containing the service response.
        """
        try:
            vlm_response = future.result()
            self.get_logger().info(f'VLM Description: {vlm_response.response}')
        except Exception as e:
            self.get_logger().error(f'Failed to get VLM response: {str(e)}')
    
    def destroy_node(self):
        """
        Clean up resources when the node is destroyed.
        
        This method ensures that the webcam is properly released.
        """
        self.cap.release()
        super().destroy_node()

def main(args=None):
    """
    The main function to run the WebcamVLMNode.

    Args:
        args: Command-line arguments (if any).
    """
    rclpy.init(args=args)
    webcam_vlm_node = WebcamVLMNode()
    rclpy.spin(webcam_vlm_node)
    webcam_vlm_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()