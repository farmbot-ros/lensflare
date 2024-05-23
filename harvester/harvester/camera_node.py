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



IP_LIGHT = {
      0 : None,
      1 : "192.168.2.251", # 7 RIGHT     70:B3:D5:DD:90:FD
      3 : "192.168.2.252", # - CENTER    70:B3:D5:DD:90:EF
      2 : "192.168.2.253", # 9 LEFT      70:B3:D5:DD:90:ED
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



class CameraNode(Node):
    def __init__(self, name, mac, device_infos):
        self.mac = mac
        self.name = name
        self.device_infos = device_infos
        self.bridge = CvBridge()
        super().__init__(f'camera_{self.name}_node')
        self.save_pub = self.create_publisher(Image, f'save_{self.name}', 10)
        self.view_pub = self.create_publisher(Image, f'view_{self.name}', 10)
        self.camera_trigger = self.create_subscription(Int16, f'trigger_{self.name}', self.callback, 10)
        
        self.buffer_bytes_per_pixel = None
        self.time_now = 0
        self.config_dir = get_package_share_directory(f'harvester') + '/configs/'


    def get_cam_info(self, msg):
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
                self.mac = camera.mac
        self.destroy_subscription(self.camera_info_sub)

    def create_camera(self):
        tries = 0
        tries_max = 6
        sleep_time_secs = 10
        result_dict = next((d for d in self.device_infos if d.get("mac") == self.mac), None)
        print(result_dict)
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
        FILE_NAME = self.config_dir + self.name + '.txt'
        print(f"Reading from file: {FILE_NAME}")
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
        # self.setup_camera(self.device)
        self.setup_camera_from_file(self.device)
        self.device.start_stream(500)

    def stop_camera(self):
        self.device.stop_stream()
        system.destroy_device(self.device)

    
    def callback(self, msg):
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
        if msg.data == 1:
            self.save_pub.publish(image_msg)
        elif msg.data == 2:
            self.view_pub.publish(image_msg)
        timedf = time.time() - self.time_now
        print(f"Time difference: {timedf}")
        self.time_now = time.time()
        self.device.requeue_buffer(buffer)




def main(args=None):
    rclpy.init()
    print('... I GOT TO GO HARVEST ...')

    # check for available cameras
    device_infos_no_dup = []
    for device in system.device_infos:
        if device not in device_infos_no_dup:
            device_infos_no_dup.append(device)
    cam_array = []
    executor = MultiThreadedExecutor(num_threads=len(device_infos_no_dup))

    for camera in device_infos_no_dup:
        mac = camera["mac"]
        hex_mac = int(mac.replace(":", ""), 16)
        try:
            camera_node = CameraNode(BY_MAC[hex_mac][2], mac, device_infos_no_dup)
            camera_node.initialize_camera()
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
            camera.stop_camera()
            executor.remove_node(camera)
        executor.shutdown()
        rclpy.shutdown()



if __name__ == '__main__':
    main()