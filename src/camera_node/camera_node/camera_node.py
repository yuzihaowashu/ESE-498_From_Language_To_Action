import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 
import cv2
import threading


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.publisher_ = self.create_publisher(Image, '/camera/image', 1)
        #self.service = self.create_service(CaptureImage, 'capture_image', self.capture_image_callback)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Could not open camera.')
            exit(1)
        self.running = True

        # start video stream
        self.video_thread = threading.Thread(target=self.video_stream, daemon=True)
        self.video_thread.start()

        self.get_logger().info('Camera node started. Streaming images...')

        timer_period = 2 # one picture per second
        self.timer = self.create_timer(timer_period, self.publish_image)

    def video_stream(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                cv2.imshow('Camera Stream', frame)
                key = cv2.waitKey(1) & 0xFF
                if key == 27:
                    self.get_logger().info('Existing camera stream ...')
                    self.shutdown()
                    break

    def publish_image(self):
        ret, frame = self.cap.read()
        if ret:
            msg = CvBridge().cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)
            self.get_logger().info('Image published.')
        else:
            self.get_logger().error('Failed to capture image from camera.')
    # def capture_image_callback(self, request, response):
    #     ret, frame = self.cap.read()
    #     if ret:
    #         response.image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
    #         self.get_logger().info('Captured image upon request.')
    #     else:
    #         self.get_logger().error('Failed to capture image.')
    #     return response

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()
    
def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()