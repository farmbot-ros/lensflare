from arena_api.system import system
from arena_api.buffer import *
import ctypes
import numpy as np
from cv_bridge import CvBridge
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image


cam_mac = { 
    "1":                ['1c:0f:af:08:49:f3', 3, 3],
    "3":                ['1c:0f:af:08:56:7d', 3, 5],
    "4":                ['1c:0f:af:08:65:e4', 3, 6],
    "7a":               ['1c:0f:af:08:57:1a', 1, 6],
    "7b":               ['1c:0f:af:08:6d:86', 1, 1],
    "7c":               ['1c:0f:af:05:44:39', 1, 5],
    "7d":               ['1c:0f:af:08:57:00', 1, 2],
    "7e":               ['1c:0f:af:08:70:d4', 1, 3],
    "9a":               ['1c:0f:af:08:49:79', 2, 1],
    "9b":               ['1c:0f:af:08:49:19', 2, 7],
    "9c":               ['1c:0f:af:08:57:23', 2, 2],
    "9d":               ['1c:0f:af:08:70:dd', 2, 5],
    "9e":               ['1c:0f:af:08:49:70', 2, 6],
    "10":               ['1c:0f:af:08:56:86', 2, 3],
    "11a":              ['1c:0f:af:08:66:81', 3, 7],
    "11b":              ['1c:0f:af:08:65:fe', 3, 7],
    "12":               ['1c:0f:af:08:49:d0', 3, 1],
    "13":               ['1c:0f:af:08:49:d9', 3, 2],
    }

ip_light = {
    1 : "192.168.2.251", # 7 RIGHT     70:B3:D5:DD:90:FD
    3 : "192.168.2.252", # - CENTER    70:B3:D5:DD:90:EF
    2 : "192.168.2.253", # 9 LEFT      70:B3:D5:DD:90:ED
}


class CameraNode(Node):
    def __init__(self, name, mac, timer, device_infos):
        self.mac = mac
        self.name = name
        self.device_infos = device_infos
        self.bridge = CvBridge()
        super().__init__(f'camera_{self.name}_node')
        self.camera_pub = self.create_publisher(Image,f'oxbo_{self.name}',10)
        self.timer = self.create_timer(timer, self.timer_callback)
        self.buffer_bytes_per_pixel = None


    def create_camera(self):
        tries = 0
        tries_max = 6
        sleep_time_secs = 10
        result_dict = next((d for d in self.device_infos if d.get("mac") == self.mac), None)
        while tries < tries_max: 
            devices = system.create_device(result_dict)
            if not devices:
                print(
                    f'Try {tries+1} of {tries_max}: waiting for {sleep_time_secs} '
                    f'secs for camera {self.name} to be connected!')
                for sec_count in range(sleep_time_secs):
                    time.sleep(1)
                    print(f'{sec_count + 1 } seconds passed ',
                        '.' * sec_count, end='\r')
                tries += 1
            else:
                print(f'Created camera {self.name}')
                return devices
        else:
            raise Exception(f'Camera {self.name} NOT found! Please connect camera {self.name} and run the example again.')

    
    def setup_camera(self, device):
        nodemap = device.nodemap
        nodes = nodemap.get_node(['Width', 'Height', 'PixelFormat', 'DeviceStreamChannelPacketSize'])
        nodes['Width'].value = nodes['Width'].max
        nodes['Height'].value = nodes['Height'].max
        nodes['PixelFormat'].value = 'BGR8'
        nodes['DeviceStreamChannelPacketSize'].value = nodes['DeviceStreamChannelPacketSize'].max
        # if nodes['DeviceStreamChannelPacketSize'].is_readable and nodes['DeviceStreamChannelPacketSize'].is_writable:
        #     print("DeviceStreamChannelPacketSize is readable and writable")
        tl_stream_nodemap = device.tl_stream_nodemap
        tl_stream_nodemap["StreamBufferHandlingMode"].value = "NewestOnly"
        tl_stream_nodemap['StreamAutoNegotiatePacketSize'].value = True
        tl_stream_nodemap['StreamPacketResendEnable'].value = True



    def initialize_camera(self):
        devices = self.create_camera()
        self.device = devices[0]
        if self.device is None: exit()
        self.num_channels = 3
        self.curr_frame_time = 0
        self.prev_frame_time = 0
        self.setup_camera(self.device)
        self.device.start_stream(500)

    def stop_camera(self):
        self.device.stop_stream()
        system.destroy_device(self.device)

    def timer_callback(self):
        print(f"-------- {self.name} --------")
        buffer = self.device.get_buffer()
        item = BufferFactory.copy(buffer)
        self.device.requeue_buffer(buffer)
        if self.buffer_bytes_per_pixel is None:
            self.image_width = item.width
            self.image_height = item.height
            self.buffer_bytes_per_pixel = int(len(item.data)/(self.image_width * self.image_height))
        array = (ctypes.c_ubyte * self.num_channels  * self.image_width * self.image_height).from_address(ctypes.addressof(item.pbytes))
        npndarray = np.ndarray(buffer=array, dtype=np.uint8, shape=(self.image_height, self.image_width, self.buffer_bytes_per_pixel))
        # array = np.asarray(item.data, dtype=np.uint8)
        # npndarray = array.reshape(self.image_height, self.image_width, self.buffer_bytes_per_pixel)
        image_msg = self.bridge.cv2_to_imgmsg(npndarray)
        self.camera_pub.publish(image_msg)
        BufferFactory.destroy(item)
    
    def timer_callback2(self):
        buffer = self.device.get_buffer(500)
        for item in buffer:
            print(f"-------- {self.name} --------")
            if self.buffer_bytes_per_pixel is None:
                self.image_width = item.width
                self.image_height = item.height
                self.buffer_bytes_per_pixel = int(len(item.data)/(self.image_width * self.image_height))
            array = (ctypes.c_ubyte * self.num_channels  * self.image_width * self.image_height).from_address(ctypes.addressof(item.pbytes))
            npndarray = np.ndarray(item=array, dtype=np.uint8, shape=(self.image_height, self.image_width, self.buffer_bytes_per_pixel))
            # array = np.asarray(item.data, dtype=np.uint8)
            # npndarray = array.reshape(self.image_height, self.image_width, self.buffer_bytes_per_pixel)
            image_msg = self.bridge.cv2_to_imgmsg(npndarray)
            self.camera_pub.publish(image_msg)
        self.device.requeue_buffer(buffer)


def main(args=None):
    rclpy.init()
    print('Hi from harvester.')
    # check for available cameras
    device_infos = system.device_infos

    try:
        camera_11a_node = CameraNode('11a','1c:0f:af:08:66:81', 0.001, device_infos)
        camera_11a_node.initialize_camera()
        camera_11b_node = CameraNode('11b','1c:0f:af:08:65:fe', 0.001, device_infos)
        camera_11b_node.initialize_camera()
        camera_13_node = CameraNode('13','1c:0f:af:08:49:d9', 0.001, device_infos)
        camera_13_node.initialize_camera()

        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(camera_11a_node)
        executor.add_node(camera_11b_node)
        executor.add_node(camera_13_node)
        try:
            executor.spin()
        finally:
            executor.shutdown()

    except Exception as e:
        print(f"An error occurred: {str(e)}")

    finally:
        camera_11a_node.stop_camera()
        camera_11b_node.stop_camera()
        camera_13_node.stop_camera()
        rclpy.shutdown()


if __name__ == '__main__':
    main()