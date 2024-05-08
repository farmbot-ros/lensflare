from arena_api.system import system
from arena_api.buffer import *
import ctypes
import numpy as np
from cv_bridge import CvBridge
import time
from harvester_interfaces.msg import CameraDevice, CameraDeviceArray
from ament_index_python.packages import get_package_share_directory

import socket
import time
import json

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import Int16


class LightController:
    def __init__(self, udp_port=6512):
        self.udp_port = udp_port

    def set_parameter_value(self, udp_ip, parameter_name, value):
        if not parameter_name: raise ValueError("Parameter name cannot be empty.")
        message = json.dumps({"jsonrpc": "2.0", "method": "setParameter", "params": {"Name": parameter_name, "Value": value}, "id": 15})
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            try:
                sock.sendto(message.encode("utf-8"), (udp_ip, self.udp_port))
                data, server_address = sock.recvfrom(self.udp_port)
                # return "param changed"
                reply_string = data.decode("utf-8")
                print(f"Commmand sent, and got reply: {reply_string}")
                # return reply_string
            except socket.error as e:
                print(f"Socket error: {e}")
                # return None

    def duty(self, udp_ip, port_nr, value):
        if not isinstance(port_nr, int) or port_nr < 1 or port_nr > 8: 
            raise ValueError("Port number must be an integer between 1 and 8.")
        if not isinstance(value, int) or value < 0 or value > 255: 
            raise ValueError("Value must be an integer between 0 and 255.")
        parameter_name = f"Port {port_nr} Scene Duty"
        return self.set_parameter_value(udp_ip, parameter_name, value)
    
    def pulse(self, udp_ip, port_nr):
        if not isinstance(port_nr, int) or port_nr < 1 or port_nr > 8:
            raise ValueError("Port number must be an integer between 1 and 4.")
        
        parameter_name = f"Send pulse port {port_nr}"
        return self.set_parameter_value(udp_ip, parameter_name, True)


class CameraNode(Node):
    def __init__(self, name, lights=False, destroy_if_none=False):
        self.name = name
        self.device_infos = []
        self.device_len = 0
        self.cam_info = None
        self.device = None
        self.bridge = CvBridge()
        super().__init__(f'camera_{self.name}_node')
        self.save_pub = self.create_publisher(Image, f'save_{self.name}', 10)
        self.view_pub = self.create_publisher(Image, f'view_{self.name}', 10)
        self.camera_trigger = self.create_subscription(Int16, f'trigger_{self.name}', self.callback, 10)
        self.camera_info_sub = self.create_subscription(CameraDeviceArray, '/caminfo', self.get_cam_info, 10)
        self.timer_initializer = self.create_timer(1, self.initialize_camera)
        self.buffer_bytes_per_pixel = None
        self.is_initialized = False
        self.destroy_if_none = destroy_if_none
        self.time_now = 0
        self.config_dir = get_package_share_directory(f'harvester') + '/configs/'
        self.lights = LightController() if lights else None


    def get_cam_info(self, msg):
        if self.device_len != len(msg.cameras):
            self.get_logger().info(f"Camera info updated.")
            system.device_infos
            self.device_len = len(msg.cameras)
            for camera in msg.cameras:
                device_info = {}
                device_info["model"] = camera.model
                device_info["vendor"] = camera.vendor
                device_info["serial"] = camera.serial
                device_info["ip"] = camera.ip
                device_info["subnetmask"] = camera.subnetmask
                device_info["defaultgateway"] = camera.defaultgateway
                device_info["mac"] = camera.mac
                device_info["name"] = camera.name
                device_info["version"] = camera.version
                device_info["dhcp"] = camera.dhcp
                device_info["presistentip"] = camera.presistentip
                device_info["lla"] = camera.lla
                self.device_infos.append(device_info)
                if camera.id == self.name:
                    self.cam_info = camera

        

    def callback(self, msg):
        if self.device is None:
            return
        # if self.lights is not None:
        #     self.lights.pulse(BY_MAC[self.mac][0], BY_MAC[self.mac][1])
        #     time.sleep(0.00001)
        buffer = self.device.get_buffer()
        if self.buffer_bytes_per_pixel is None:
            self.image_width = buffer.width
            self.image_height = buffer.height
            self.num_channels = 3
            self.buffer_bytes_per_pixel = int(len(buffer.data)/(self.image_width * self.image_height))
        array = (ctypes.c_ubyte * (self.num_channels  * self.image_width * self.image_height)).from_address(ctypes.addressof(buffer.pbytes))
        npndarray = np.ctypeslib.as_array(array).reshape(self.image_height, self.image_width, self.num_channels)
        # image_msg = self.bridge.cvtype2_to_dtype_with_channels(array)
        image_msg = self.bridge.cv2_to_imgmsg(npndarray) 
        # image_msg = self.bridge.cv2_to_compressed_imgmsg(npndarray)
        image_msg.header.stamp = self.get_clock().now().to_msg()
        if msg.data == 1:
            self.save_pub.publish(image_msg)
        elif msg.data == 2:
            self.view_pub.publish(image_msg)
        timedf = time.time() - self.time_now
        print(f"Time difference: {timedf}")
        self.time_now = time.time()
        self.device.requeue_buffer(buffer)


    def setup_camera_from_file(self, device):
        self.get_logger().info(f"Setting up camera {self.name}, from file.")
        FILE_NAME = self.config_dir + self.name + '.txt'
        print(f"Reading from file: {FILE_NAME}")
        device.nodemap.read_streamable_node_values_from(FILE_NAME)
        
    def initialize_camera(self):
        while self.cam_info is None:
            self.get_logger().info(f"Camera {self.name} not initialized yet.")
            return
        result_dict = next((d for d in self.device_infos if d.get("mac") == self.cam_info.mac), None)
        devices = system.create_device(result_dict)
        self.device = devices[0]

        self.setup_camera_from_file(self.device)
        self.device.start_stream(500)
        self.destroy_timer(self.camera_timer)
        self.is_initialized 



def main(args=None):
    rclpy.init()
    print('... I GOT TO GO HARVEST ...')

    cameras = ["7a", "7b", "7c", "7d", "7e", "9a", "9b", "9c", "9d", "9e", "1", "3", "4", "10", "11a", "11b", "12", "13"]
    cameras = ["11a"]
    cam_array = []cam_info
    executor = MultiThreadedExecutor(num_threads=len(cameras))

    for camera in cameras:
        try:
            camera_node = CameraNode(camera, lights=False)
            cam_array.append(camera_node)
            try:
                executor.add_node(camera_node)
            except Exception as e:
                print(f"An error occurred: {str(e)}")
        except Exception as e:
            print(f"An error occurred: {str(e)}")

    try:
        executor.spin()
    except Exception as e:
        print(f"An error occurred: {str(e)}")
    finally:
        for camera in cam_array:
            # camera.stop_camera() 
            executor.remove_node(camera)
        executor.shutdown()
        rclpy.shutdown()



if __name__ == '__main__':
    main()