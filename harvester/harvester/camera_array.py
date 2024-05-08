from arena_api.system import system
from arena_api.buffer import *
from harvester_interfaces.msg import CameraDevice, CameraDeviceArray
import ctypes
import numpy as np
import cv2
from cv_bridge import CvBridge
import time
import socket
import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraInfo(Node):
    def __init__(self, device_info):
        super().__init__('camera_node')
        self.devices_info = device_info
        self.camera_info = None
        self.timer_publish = self.create_timer(1, self.publish_array_info)
        self.array_pub = self.create_publisher(CameraDeviceArray, '/caminfo', 10)
        for device in self.devices_info:
            self.get_logger().info(f"Device {device['mac']} info are published")

    """
    A list of dictionaries used to create devices. Each dictionary represents a discovered device on the network.
    A device info dictionary has the following keys: `model`,`vendor`, `serial`, `ip`, `subnetmask`,
    `defaultgateway`, `mac`, `name`, `dhcp`, `presistentip`, `lla`, and `version`. 
    """
    def publish_array_info(self):
        if self.devices_info is None:
            self.array_pub.publish(camera_array)
            return
    
        camera_array = CameraDeviceArray()
        for device in self.devices_info:
            camera_device = CameraDevice()
            camera_device.model = device["model"]
            camera_device.vendor = device["vendor"]
            camera_device.serial = device["serial"]
            camera_device.ip = device["ip"]
            camera_device.subnetmask = device["subnetmask"]
            camera_device.defaultgateway = device["defaultgateway"]
            camera_device.mac = device["mac"]
            camera_device.name = device["name"]
            camera_device.dhcp = device["dhcp"]
            camera_device.presistentip = device["presistentip"]
            camera_device.lla = device["lla"]
            camera_device.version = device["version"]
            camera_array.cameras.append(camera_device)
        # self.
        

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraInfo()
    rclpy.spin(camera_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
