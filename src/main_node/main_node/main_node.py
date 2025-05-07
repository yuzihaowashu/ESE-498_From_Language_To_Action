import rclpy
from rclpy.node import Node
from robot_interfaces.srv import DetectObject

class MainNode(Node):

    def __init__(self):
        super().__init__('main_node')

        self.client = self.create_client(DetectObject, 'detect_object_service')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for detection service...')

        self.timer = self.create_timer(10.0, self.trigger_detection)

    def trigger_detection(self):
        prompt = input("Enter detection instruction (e.g., 'Place the red block on the green logo'): ")

        request = DetectObject.Request()
        request.prompt = prompt

        self.get_logger().info(f"Sending detection request: {prompt}")
        future = self.client.call_async(request)
        future.add_done_callback(self.handle_detection_result)
    
    def handle_detection_result():
        result = future.result()
        if result.success:
            self.get_logger().info(f"Detection successfully: {result.result_json}")
        else:
            self.get_logger().error("Detection failed. ")

def main(args=None):
    rclpy.init(args=args)
    node = MainNode()
    rclpy.spin(node)
    node.destory_node()
    rclpy.shutdown()

if __name__ == 'main':
    main()