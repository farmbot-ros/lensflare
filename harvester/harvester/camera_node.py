from arena_api.system import system
from arena_api.buffer import *
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


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node') # TODO: include camera name here
        self.declare_parameter('camera_name','13')
        self.declare_parameter('scan_cameras',True)

        self.cam = self.get_parameter('camera_name').get_parameter_value().string_value
        if self.get_parameter('scan_cameras').get_parameter_value().bool_value:
            system.device_infos
        else:
            time.sleep(30)
            system.device_infos

        print(self.cam)
        self.bridge = CvBridge()

        self.mac = { 
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

        self.ip_light = {
            1 : "192.168.1.7", # 7 RIGHT     70:B3:D5:DD:90:FD
            3 : "192.168.1.8", # - CENTER    70:B3:D5:DD:90:EF
            2 : "192.168.1.9", # 9 LEFT      70:B3:D5:DD:90:ED
        }

        self.camera_pub = self.create_publisher(Image,f'camera_{self.cam}',10)

        self.example_entry_point()


    def create_devices_with_tries(self):
        tries = 0
        tries_max = 6
        sleep_time_secs = 10
        sliced_info_dict = {'mac': self.mac[self.cam][0]}
        
        while tries < tries_max:  # Wait for device for 60 seconds
            devices = system.create_device(sliced_info_dict)
            # spec_device = system.create_device()
            if not devices:
                print(
                    f'Try {tries+1} of {tries_max}: waiting for {sleep_time_secs} '
                    f'secs for a device to be connected!')
                for sec_count in range(sleep_time_secs):
                    time.sleep(1)
                    print(f'{sec_count + 1 } seconds passed ',
                        '.' * sec_count, end='\r')
                tries += 1
            else:
                print(f'Created {len(devices)} device(s)')
                return devices
        else:
            raise Exception(f'No device found! Please connect a device and run '
                            f'the example again.')
        

    def setup(self, device):
        nodemap = device.nodemap
        nodes = nodemap.get_node(['Width', 'Height', 'PixelFormat'])
        nodes['Width'].value = nodes['Width'].max
        nodes['Height'].value = nodes['Height'].max
        nodes['PixelFormat'].value = 'BGR8'
        num_channels = 3
        tl_stream_nodemap = device.tl_stream_nodemap
        tl_stream_nodemap["StreamBufferHandlingMode"].value = "NewestOnly"
        tl_stream_nodemap['StreamAutoNegotiatePacketSize'].value = True
        tl_stream_nodemap['StreamPacketResendEnable'].value = True
        return num_channels


    def example_entry_point(self):
        devices = self.create_devices_with_tries()
        device = devices[0]
        # for dev in devices:
        #     if dev.nodemap.get_node("GevMACAddress").value == self.mac[self.cam][0]:
        #         device = dev
        #     else:
        #         print('not same device')

        print(self.mac[self.cam][2])
        # light = LightController(6512)
        if device is None: exit()
        # print(light.duty(ip_light[mac[cam][1]], mac[cam][2], 250))
        num_channels = self.setup(device)
        curr_frame_time = 0
        prev_frame_time = 0

        with device.start_stream():
            count = 0
            while True:
                count += 1
                curr_frame_time = time.time()
                buffer = device.get_buffer()
                item = BufferFactory.copy(buffer)
                device.requeue_buffer(buffer)
                buffer_bytes_per_pixel = int(len(item.data)/(item.width * item.height))
                array = (ctypes.c_ubyte * num_channels * item.width * item.height).from_address(ctypes.addressof(item.pbytes))
                npndarray = np.ndarray(buffer=array, dtype=np.uint8, shape=(item.height, item.width, buffer_bytes_per_pixel))
                fps = str(1/(curr_frame_time - prev_frame_time))
                cv2.putText(npndarray, fps, (7, 70), cv2.FONT_HERSHEY_SIMPLEX, 3, (100, 255, 0), 3, cv2.LINE_AA)
                # cv2.namedWindow("Lucid", cv2.WINDOW_NORMAL) 
                # cv2.imshow('Lucid', npndarray)
                image_msg = self.bridge.cv2_to_imgmsg(npndarray)
                self.camera_pub.publish(image_msg)
                BufferFactory.destroy(item)
                prev_frame_time = curr_frame_time
                key = cv2.waitKey(1)
                if key == 27:
                    break  
            device.stop_stream()
            # cv2.destroyAllWindows()
        # print(light.duty(ip_light[mac[cam][1]], mac[cam][2], 0))
        system.destroy_device()


def main(args=None):
    print('Hi from harvester.')
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
