import rclpy
from rclpy.node import Node
from robot_interfaces.msg import ObjectDetection, ExecutionComplete
import time
from pymycobot import MyCobot, PI_PORT, PI_BAUD
import numpy as np
import cv2

class ExecutionNode(Node):
    def __init__(self):
        super().__init__('execution_node')
        self.subscription = self.create_subscription(
            ObjectDetection,
            'detected_objects',
            self.listener_callback,
            10
        )
        self.subscription

        self.execution_complete_pub = self.create_publisher(ExecutionComplete, 'execution_complete', 10)


        # Initialize myCobot
        self.arm = MyCobot(PI_PORT, PI_BAUD)
        self.plane_height = 80
        self.fixed_z = 200
        self.default_pose = [111, -60, 200, -179.93, -0.18, -140.29]
        self.fixed_rotation = [-179.93, -0.18, -140.29]
        self.get_logger().info("Execution node initialized and listening for detected objects. ") 

        # im: image/camera      mc: robot arm/world frame
        self.pts_image = np.array([
            [68, 430],
            [570, 45],
            [313, 428]
        ], dtype=np.float32)

        self.pts_robot = np.array([
            [115, -2], 
            [200, -120], 
            [112.5, -58.8]
        ], dtype=np.float32)
        self.M = cv2.getAffineTransform(self.pts_image, self.pts_robot)

    def eye2hand(self, x_im, y_im):
        pt = np.array([[[x_im, y_im]]], dtype=np.float32)
        out = cv2.transform(pt, self.M)
        coords = tuple(out[0][0])
        return coords[0], coords[1]

    def listener_callback(self, msg):
        start_x_im = (msg.start_xmin + msg.start_xmax) // 2
        start_y_im = (msg.start_ymin + msg.start_ymax) // 2

        end_x_im = (msg.end_xmin + msg.end_xmax) // 2
        end_y_im = (msg.end_ymin + msg.end_ymax) // 2

        start_x_mc, start_y_mc = self.eye2hand(start_x_im, start_y_im)
        end_x_mc, end_y_mc = self.eye2hand(end_x_im, end_y_im)

        grab_point = [start_x_mc, start_y_mc, self.fixed_z, self.fixed_rotation[0], self.fixed_rotation[1], self.fixed_rotation[2]+msg.rotation_angle_start]
        place_point = [end_x_mc, end_y_mc, self.fixed_z, self.fixed_rotation[0], self.fixed_rotation[1], self.fixed_rotation[2]+msg.rotation_angle_end]

        self.get_logger().info(f'From {msg.start_object} -> {msg.end_object}')  

        self.execute_movement(grab_point, place_point)

    def execute_movement(self, grab_point, place_point): 
        arm = self.arm
        
        arm.set_gripper_state(0, 100) # open gripper
        time.sleep(1)
        arm.send_coords(grab_point, 100)
        time.sleep(2)
        arm.send_coords([grab_point[0], grab_point[1], self.plane_height, *grab_point[3:]], 100) # Move down to object
        time.sleep(2)
        arm.set_gripper_state(1, 100) # close gripper
        time.sleep(1)
        arm.send_coords(grab_point, 100) # lift up
        time.sleep(2)

        # move above drop point
        arm.send_coords(place_point, 100)
        time.sleep(2)
        arm.send_coords([place_point[0], place_point[1], self.plane_height, *place_point[3:]], 100) # Move down
        time.sleep(2)
        arm.set_gripper_state(0, 100) # open gripper
        time.sleep(1)
        arm.send_coords(place_point, 100) # back up
        time.sleep(2)

        self.get_logger().info("Pick-and-place operation complete. ")
        # Move to the default pose
        arm.send_coords(self.default_pose, 100)
        time.sleep(3)

        # Send finished message
        complete_msg = ExecutionComplete()
        complete_msg.finished = True
        self.execution_complete_pub.publish(complete_msg)
        self.get_logger().info("Robot execution complete message published. ")

def main(args=None):
    rclpy.init(args=args)
    node = ExecutionNode()
    rclpy.spin(node)
    node.destory_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

