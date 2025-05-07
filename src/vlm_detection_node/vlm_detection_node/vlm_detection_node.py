import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 
import cv2
import os

from robot_interfaces.srv import DetectObject
from robot_interfaces.msg import ObjectDetection, ExecutionComplete
from .vlm_api import find_coords
from .frame_label_api import get_rotation_angle
import tempfile
import json
from .visualize_box import visualize_detections
from .successful_placement_test import is_successful_placement
import time

class VLMDetectionNode(Node):
    def __init__(self):
        super().__init__('vlm_detection_node')

        self.bridge = CvBridge()
        self.latest_image = None

        self.execution_complete_sub = self.create_subscription(
            ExecutionComplete, 'execution_complete', self.execution_complete_callback, 10
        )
        self.pending_task = False
        self.retry_count = 0
        self.max_retries = 0
        self.current_prompt = ""
        self.current_detection = None

        self.publisher_ = self.create_publisher(ObjectDetection, 'detected_objects', 10)

        self.subscription = self.create_subscription(Image, '/camera/image', self.image_callback, 1)

        self.detection_service = self.create_service(DetectObject, 'detect_object_service', self.service_callback)

        self.get_logger().info('Object Detection Node started and subscribed to /camera/image')

    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        height, width, channels = self.latest_image.shape
        self.get_logger().info(f"Received image: {width}x{height}, Channels: {channels}")  

    def execution_complete_callback(self, msg):
        if msg.finished and self.pending_task:
            self.get_logger().info("Execution finished, verifying placement...")
            self.robot_busy = False
            self.verify_placement()
    
    def verify_placement(self):

        with tempfile.NamedTemporaryFile(suffix='.jpg', delete=False) as tmp:
            img_path = tmp.name 
            cv2.imwrite(img_path, self.latest_image)

        detection_result_after = self.current_detection

        success, distance = is_successful_placement(
            detection_result_after["start_coordinates"][0],
            detection_result_after["end_coordinates"][0]
        )
        self.get_logger().info(f"Distance between object and target: {distance:.2f} pixels")
        if success:
            self.get_logger().info("Placement successful! ")
            self.pending_task = False
        else: 
            self.get_logger().warn("Placement unsuccessful, retrying...")
            self.retry_placement()
    
    def retry_placement(self):
        if self.retry_count < self.max_retries:
            self.retry_count += 1
            self.publish_detection()
        else:
            self.get_logger().error("Maximum retries reached. ")
            self.pending_task = False


    def publish_detection(self): 
        with tempfile.NamedTemporaryFile(suffix='.jpg', delete=False) as tmp:
            img_path = tmp.name 
            cv2.imwrite(img_path, self.latest_image)

            # vlm_api
            detection_result = find_coords(img_path, prompt=self.current_prompt)
            self.current_detection = detection_result
            # visualization
            current_image = cv2.imread(img_path)
            visualize_detections(current_image, detection_result)

            os.unlink(img_path)

            
            msg = ObjectDetection()
            msg.start_object = detection_result.get("start", "Unknown")
            if "start_coordinates" in detection_result and len(detection_result["start_coordinates"]) > 0: 
                msg.start_xmin, msg.start_ymin, msg.start_xmax, msg.start_ymax = detection_result["start_coordinates"][0] 

            msg.end_object = detection_result.get("end", "Unknown")
            if "end_coordinates" in detection_result and len(detection_result["end_coordinates"]) > 0:
                msg.end_xmin, msg.end_ymin, msg.end_xmax, msg.end_ymax = detection_result["end_coordinates"][0]
            
            # frame_label
            bbox_width_start = msg.start_xmax - msg.start_xmin
            bbox_height_start = msg.start_ymax - msg.start_ymin
            if max(bbox_width_start, bbox_height_start) > 200: 
                angle_start = get_rotation_angle(
                    image=self.latest_image, 
                    xmin=msg.start_xmin,
                    ymin=msg.start_ymin,
                    xmax=msg.start_xmax,
                    ymax=msg.start_ymax
                )
                msg.rotation_angle_start = float(angle_start)
            else: 
                msg.rotation_angle_start = 0.0

            bbox_width_end = msg.end_xmax - msg.end_xmin
            bbox_height_end = msg.end_ymax - msg.end_ymin
            if max(bbox_width_end, bbox_height_end) > 200: 
                angle_end = get_rotation_angle(
                    image=self.latest_image, 
                    xmin=msg.end_xmin,
                    ymin=msg.end_ymin,
                    xmax=msg.end_xmax,
                    ymax=msg.end_ymax
                )
                msg.rotation_angle_end = float(angle_end)
            else: 
                msg.rotation_angle_end = 0.0
            
            self.message = msg
            self.publisher_.publish(msg)


    def service_callback(self, request, response):
        self.current_prompt = request.prompt
        self.retry_count = 0
        self.pending_task = True
        self.publish_detection()

        response.success = True
        return response



def main(args=None):
    rclpy.init(args=args)
    node = VLMDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()