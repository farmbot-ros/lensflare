from arena_api.system import system
from arena_api.buffer import *
import ctypes
import numpy as np
from cv_bridge import CvBridge
import time
from harvester_interfaces.msg import CameraDevice, CameraDeviceArray

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image



IP_LIGHT = {
      0 : None,
      1 : "192.168.1.7", # 7 RIGHT     70:B3:D5:DD:90:FD
      3 : "192.168.1.8", # - CENTER    70:B3:D5:DD:90:EF
      2 : "192.168.1.9", # 9 LEFT      70:B3:D5:DD:90:ED
}

GAIN = 20
EXP = 1.2

BY_MAC = { 
    ##  MAC             IP      CHANNEL NAME    EXP     GAIN
    30853686642969: [IP_LIGHT[2],   7,   "9b",   521*EXP,  30],
    30853686643065: [IP_LIGHT[2],   1,   "9a",   521*EXP,  30],
    30853686646563: [IP_LIGHT[2],   2,   "9c",   521*EXP,  30],
    30853686653149: [IP_LIGHT[2],   5,   "9d",   521*EXP,  30],
    30853686643056: [IP_LIGHT[2],   6,   "9e",   521*EXP,  30],
    30853686646554: [IP_LIGHT[1],   6,   "7a",   521*EXP,  30],
    30853686652294: [IP_LIGHT[1],   1,   "7b",   521*EXP,  30],
    30853686445113: [IP_LIGHT[1],   5,   "7c",   521*EXP,  30],
    30853686646528: [IP_LIGHT[1],   2,   "7d",   521*EXP,  30],
    30853686653140: [IP_LIGHT[1],   3,   "7e",   521*EXP,  30],
    30853686643187: [IP_LIGHT[0],   0,   "1",   2847*1.1,  20],
    30853686650340: [IP_LIGHT[3],   6,   "4",    528*2.8,  30],
    30853686646397: [IP_LIGHT[3],   5,   "3",    810*2.8,  20],
    30853686643161: [IP_LIGHT[3],   2,   "13",   805*3.8,  20],
    30853686643152: [IP_LIGHT[3],   1,   "12",   460*3.8,  40],
    30853686646406: [IP_LIGHT[3],   3,   "10",     867.0,  30],
    30853686650497: [IP_LIGHT[3],   7,   "11a",    498.0,  15],
    30853686650366: [IP_LIGHT[3],   7,   "11b",    498.0,  15],
}

BY_NAME = { 
    "9b":   [30853686642969, 2, 7],   #1c:0f:af:08:49:19
    "9a":   [30853686643065, 2, 1],   #1c:0f:af:08:49:79
    "9c":   [30853686646563, 2, 2],   #1c:0f:af:08:57:23
    "9d":   [30853686653149, 2, 5],   #1c:0f:af:08:70:dd
    "9e":   [30853686643056, 2, 6],   #1c:0f:af:08:49:70
    "7a":   [30853686646554, 1, 6],   #1c:0f:af:08:57:1a
    "7b":   [30853686652294, 1, 1],   #1c:0f:af:08:6d:86
    "7c":   [30853686445113, 1, 5],   #1c:0f:af:05:44:39
    "7d":   [30853686646528, 1, 2],   #1c:0f:af:08:57:00
    "7e":   [30853686653140, 1, 3],   #1c:0f:af:08:70:d4
    "1":    [30853686643187, 3, 3],   #1c:0f:af:08:49:f3
    "4":    [30853686650340, 3, 6],   #1c:0f:af:08:65:e4
    "3":    [30853686646397, 3, 5],   #1c:0f:af:08:56:7d
    "13":   [30853686643161, 3, 2],   #1c:0f:af:08:49:d9
    "12":   [30853686643152, 3, 1],   #1c:0f:af:08:49:d0
    "10":   [30853686646406, 2, 3],   #1c:0f:af:08:56:86
    "11a":  [30853686650497, 3, 7],   #1c:0f:af:08:66:81
    "11b":  [30853686650366, 3, 7],   #1c:0f:af:08:65:fe
}


class CameraNode(Node):
    def __init__(self, name, mac, timer, device_infos):
        self.mac = mac
        self.name = name
        self.device_infos = device_infos
        self.bridge = CvBridge()
        super().__init__(f'camera_{self.name}_node')
        self.camera_pub = self.create_publisher(Image, f'oxbo_{self.name}', 10)
        self.timer = self.create_timer(timer, self.timer_callback)
        self.buffer_bytes_per_pixel = None
        self.time_now = 0


    def create_camera(self):
        tries = 0
        tries_max = 6
        sleep_time_secs = 10
        result_dict = next((d for d in self.device_infos if d.get("mac") == self.mac), None)
        while tries < tries_max: 
            devices = system.create_device(result_dict)
            if not devices:
                print(f'Try {tries}: Waiting for camera {self.name} to be connected!')
                for sec_count in range(sleep_time_secs):
                    time.sleep(1)
                tries += 1
            else:
                print(f'Created camera {self.name}')
                return devices
        else:
            raise Exception(f'Camera {self.name} NOT found! Please connect camera {self.name} and run the example again.')

    def setup_camera_from_file(self, device):
        self.get_logger().info(f"Setting up camera {self.name}, from file.")
        FILE_NAME = '/home/bresilla/ROS_OXBO/src/harvester/configs/' + self.name + '.txt'
        device.nodemap.read_streamable_node_values_from(FILE_NAME)

    def setup_camera(self, device):
        nodemap = device.nodemap
        self.mac_add = nodemap.get_node('GevMACAddress').value
        nodemap.get_node('Width').value = nodemap.get_node('Width').max
        nodemap.get_node('Height').value = nodemap.get_node('Height').max
        nodemap.get_node('PixelFormat').value = 'BGR8'
        nodemap.get_node('DeviceStreamChannelPacketSize').value = nodemap.get_node('DeviceStreamChannelPacketSize').max
        nodemap.get_node("ExposureAutoLimitAuto").value = "Off"
        nodemap.get_node("ExposureAutoUpperLimit").value = BY_MAC[self.mac_add][3]
        nodemap.get_node("GainAuto").value = "Continuous"
        nodemap.get_node("GainAutoUpperLimit").value = float(BY_MAC[self.mac_add][4])

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
        # self.setup_camera_from_file(self.device)
        self.device.start_stream(500)

    def stop_camera(self):
        self.device.stop_stream()
        system.destroy_device(self.device)

    
    def timer_callback(self):
        buffer = self.device.get_buffer()
        if self.buffer_bytes_per_pixel is None:
            self.image_width = buffer.width
            self.image_height = buffer.height
            self.buffer_bytes_per_pixel = int(len(buffer.data)/(self.image_width * self.image_height))
        array = (ctypes.c_ubyte * (self.num_channels  * self.image_width * self.image_height)).from_address(ctypes.addressof(buffer.pbytes))
        npndarray = np.ctypeslib.as_array(array).reshape(self.image_height, self.image_width, self.num_channels)
        # image_msg = self.bridge.cvtype2_to_dtype_with_channels(array)
        image_msg = self.bridge.cv2_to_imgmsg(npndarray) 
        # image_msg = self.bridge.cv2_to_compressed_imgmsg(npndarray)
        image_msg.header.stamp = self.get_clock().now().to_msg()
        self.camera_pub.publish(image_msg)
        timedf = time.time() - self.time_now
        print(f"Time difference: {timedf}")
        self.time_now = time.time()
        self.device.requeue_buffer(buffer)


class CameraInfo(Node):
    def __init__(self, device_info):
        super().__init__('camera_node')
        self.devices_info = device_info
        print(self.devices_info)
        self.timer_publish = self.create_timer(1, self.publish_array_info)
        self.array_pub = self.create_publisher(CameraDeviceArray, '/oxbo/cam_infos', 10)

    def publish_array_info(self):
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
            camera_device.version = device["version"]
            camera_device.dhcp = device["dhcp"]
            camera_device.presistentip = device["presistentip"]
            camera_device.lla = device["lla"]
            camera_array.cameras.append(camera_device)
        self.array_pub.publish(camera_array)


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
        # camera_13_node = CameraNode('13','1c:0f:af:08:49:d9', 0.001, device_infos)
        # camera_13_node.initialize_camera()
        # camera_test_node = CameraNode('test','1c:0f:af:08:66:A4', 0.001, device_infos)
        # camera_test_node.initialize_camera()

        device_info_node = CameraInfo(device_infos)

        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(camera_11a_node)
        executor.add_node(camera_11b_node)
        # executor.add_node(camera_13_node)
        # executor.add_node(camera_test_node)
        executor.add_node(device_info_node)
        try:
            executor.spin()
        finally:
            executor.shutdown()

    except Exception as e:
        print(f"An error occurred: {str(e)}")

    finally:
        camera_11a_node.stop_camera()
        camera_11b_node.stop_camera()
        # camera_13_node.stop_camera()
        # camera_test_node.stop_camera()
        rclpy.shutdown()


if __name__ == '__main__':
    main()