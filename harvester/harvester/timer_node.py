import os
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int16
import time


class TimerNode(Node):
    def __init__(self, inference_cameras, acquisition_cameras):
        super().__init__('timer_node')
        self.trigger_pub_dict = {}
        self.inference_cameras = inference_cameras
        self.acquisition_cameras = acquisition_cameras

        for camera in self.inference_cameras:
            print(camera)
            self.trigger_pub_dict[camera] = self.create_publisher(Int16, f"trigger_{camera}",10)

        for camera in self.acquisition_cameras:
            print(camera)
            self.trigger_pub_dict[camera] = self.create_publisher(Int16, f"trigger_{camera}",10)

        self.inference_timer = self.create_timer(6,self.inference_timer_callback)
        self.acquisition_timer = self.create_timer(18,self.acquisition_timer_callback)


    def inference_timer_callback(self):
        for camera in self.inference_cameras:
            trigger_msg = Int16()
            trigger_msg.data = 1
            self.trigger_pub_dict[camera].publish(trigger_msg)
            print(f"hi from {camera}")
            time.sleep(0.1)


    def acquisition_timer_callback(self):
        for camera in self.acquisition_cameras:
            trigger_msg = Int16()
            trigger_msg.data = 1
            self.trigger_pub_dict[camera].publish(trigger_msg)
            print(f"hi from {camera}")
            time.sleep(0.1)


def main():
    rclpy.init()
    inference_cameras = ["11a", "11b", "13"]
    acquisition_cameras = ["7a", "7b", "7c", "7d", "7e", "9a", "9b", "9c", "9d", "9e", "1", "3", "4", "10", "12"]
    node = TimerNode(inference_cameras, acquisition_cameras)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
