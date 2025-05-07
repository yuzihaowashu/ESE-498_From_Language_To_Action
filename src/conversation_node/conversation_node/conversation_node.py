import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from robot_interfaces.srv import DetectObject
from .vlm_api import query_vlm_with_image, vlm_vqa
import json
import tempfile
import cv2
import threading
import requests
import time

USE_VOICE_INPUT = True
LAPTOP_IP = "http://xxxxxxxx:8000" # Your laptop's ip address

class ConversastionNode(Node):
    def __init__(self): 
        super().__init__('conversation_node')

        # listen for image
        self.bridge = CvBridge() 
        self.latest_image = None
        self.subscription = self.create_subscription(
            Image, 
            '/camera/image', 
            self.image_callback, 
            10
        )

        self.awaiting_user_input = True
        self.pending_task = False

        self.vlm_client = self.create_client(DetectObject, 'detect_object_service')
        self.get_logger().info("Converstaion node ready: ")
        self.get_logger().info("Waiting for first image... ")
        while rclpy.ok() and self.latest_image is None: 
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("first image received")

        if USE_VOICE_INPUT: 
            self.voice_thread = threading.Thread(target=self.voice_input_loop)
            self.voice_thread.daemon = True
            self.voice_thread.start()
        else: 
            self.input_thread = threading.Thread(target=self.input_loop)
            self.input_thread.daemon = True
            self.input_thread.start()

    def image_callback(self, msg): 
        try: 
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e: 
            self.get_logger().error(f"cv bridge conversation failed: {e}")

    def input_loop(self): 
        while rclpy.ok():

            if not self.awaiting_user_input: 
                time.sleep(0.5)
                continue
            try:
                user_input = input("User: ")
                self.handle_input(user_input) 
            except KeyboardInterrupt: 
                break

    def voice_input_loop(self): 
        while rclpy.ok():

            if not self.awaiting_user_input: 
                time.sleep(0.5)
                continue

            try: 
                #self.send_tts("I am ready. Feel free to talk with me.")

                self.get_logger().info("Requesting voice input from laptop...") 
                response = requests.get(f"{LAPTOP_IP}/listen", timeout=10)
                self.get_logger().info(f"Raw/listen response: {response.json()}")
                if response.status_code == 200:
                    raw_transcript = response.json().get("transcript", "")
                    if isinstance(raw_transcript, str): 
                        transcript = raw_transcript.strip()
                    else: 
                        transcript = ""
                    if transcript: 
                        self.awaiting_user_input = False
                        self.get_logger().info(f"Voice input: {transcript}")
                        if isinstance(transcript, str):
                            transcript = transcript.strip()
                            self.awaiting_user_input = False
                            self.handle_input(transcript)
                        else: 
                            self.get_logger().error(f"transcript is not a string")
                            self.awaiting_user_input = True

            except Exception as e: 
                self.get_logger().warn(f"Error during voice input request: {e}")


    def handle_input(self, user_input): 
        with tempfile.NamedTemporaryFile(suffix='.jpg', delete=False) as tmp:
            img_path = tmp.name 
            cv2.imwrite(img_path, self.latest_image)

        try: 
            response = query_vlm_with_image(img_path, user_input)
            self.awaiting_user_input = False
            self.pending_task = True
            self.process_llm_response(response) 
        except Exception as e: 
            self.get_logger().error(f"Failed to process input: {e}")

    def process_llm_response(self, response_json): 

        parsed = response_json
 
        self.get_logger().info(parsed.get('response'))

        self.send_tts(parsed.get("response", ""))

        with tempfile.NamedTemporaryFile(suffix='.jpg', delete=False) as tmp:
            img_path = tmp.name 
            cv2.imwrite(img_path, self.latest_image)

        no_move_command = True
        for func_call in parsed.get("function", []): 
            if func_call.startswith("vlm_move("): 
                no_move_command = False
                text_arg = func_call[len("vlm_move("):-1].strip("'\"")
                self.call_vlm_move_service(text_arg)
            elif func_call.startswith("vlm_vqa("): 
                question = func_call[len("vlm_vqa("):-1].strip("'\"")
                vqa_result = vlm_vqa(img_path, question)
                self.send_tts({vqa_result}) 
        
        if no_move_command: 
            self.awaiting_user_input = True

    def call_vlm_move_service(self, prompt_text): 
        while not self.vlm_client.wait_for_service(timeout_sec=1.0): 
            self.get_logger().warn("Waiting for vlm_detection_node...")
        request = DetectObject.Request()
        request.prompt = prompt_text
        future = self.vlm_client.call_async(request)
        future.add_done_callback(self.vlm_response_callback)

        if future.result().success: 
            self.get_logger().info("vlm_move completed. ")
        else: 
            self.get_logger().error("vlm_move failed. ")
        
    def vlm_response_callback(self, future): 
        try: 
            result = future.result()
            if result is not None and result.success: 
                self.get_logger().info("vlm_move completed. ")
                self.send_tts("Done! ")
            else: 
                self.get_logger().error("vlm_move failed. ")
        except Exception as e: 
            self.get_logger().error(f"vlm_move callback error: {e}")
        finally: 
            self.pending_task = False
            self.awaiting_user_input = True

    def send_tts(self, text): 
        # if not text.strip():
        #     return 
        print(f"TTS: {text}")
        if USE_VOICE_INPUT: 
            try: 
                requests.post(f"{LAPTOP_IP}/speak", data={"text": text})
            except Exception as e:
                self.get_logger().warn(f"TTS request failed: {e}")

def main(args=None): 
    rclpy.init(args=args)
    node = ConversastionNode()

    rclpy.spin(node)
    node.destroy_node() 
    rclpy.shutdown() 

if __name__ == '__main__': 
    main()