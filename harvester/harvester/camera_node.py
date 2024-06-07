from arena_api.system import system
from arena_api.buffer import *
import ctypes
import numpy as np
import cv2
from cv_bridge import CvBridge
import time
from harvester_interfaces.msg import CameraDevice, CameraDeviceArray
from harvester_interfaces.srv import TriggerCamera
from ament_index_python.packages import get_package_share_directory

import socket
import time
import json

from std_msgs.msg import Int16, Bool
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import Int16, Float32



IP_LIGHT = {
      0 : None,
      1 : "192.168.2.251", # 7 RIGHT     70:B3:D5:DD:90:FD
      2 : "192.168.2.253", # 9 LEFT      70:B3:D5:DD:90:ED
      3 : "192.168.2.252", # - CENTER    70:B3:D5:DD:90:EF
}

GAIN = 20
EXP = 1.2

BY_MAC = { 
    ##  MAC             IP      CHANNEL NAME    EXP     GAIN    SW     SCPD        SCFTD
    30853686642969: [IP_LIGHT[2],   7,   "9b",   521*EXP,  30,   1,    240000,    0*80000],
    30853686643065: [IP_LIGHT[2],   1,   "9a",   521*EXP,  30,   1,    240000,    1*80000],
    30853686646563: [IP_LIGHT[2],   2,   "9c",   521*EXP,  30,   1,    240000,    2*80000],
    30853686653149: [IP_LIGHT[2],   5,   "9d",   521*EXP,  30,   1,    240000,    3*80000],
    30853686643056: [IP_LIGHT[2],   6,   "9e",   521*EXP,  30,   2,    240000,    0*80000],
    30853686646554: [IP_LIGHT[1],   6,   "7a",   521*EXP,  30,   2,    240000,    1*80000],
    30853686653140: [IP_LIGHT[1],   3,   "7e",   521*EXP,  30,   2,    240000,    2*80000],
    30853686445113: [IP_LIGHT[1],   5,   "7c",   521*EXP,  30,   2,    240000,    3*80000],
    30853686646528: [IP_LIGHT[1],   2,   "7d",   521*EXP,  30,   3,    240000,    0*80000],
    30853686652294: [IP_LIGHT[1],   1,   "7b",   521*EXP,  30,   3,    240000,    2*80000],
    30853686643187: [IP_LIGHT[0],   0,   "1",   2847*1.1,  20,   3,    240000,    1*80000],
    30853686650340: [IP_LIGHT[3],   6,   "4",    528*2.8,  30,   3,    240000,    3*80000],
    30853686646397: [IP_LIGHT[3],   5,   "3",    810*2.8,  20,   4,    240000,    0*80000],
    30853686643152: [IP_LIGHT[3],   1,   "12",   460*3.8,  40,   4,    240000,    1*80000],
    30853686646406: [IP_LIGHT[3],   3,   "10",     867.0,  30,   4,    240000,    2*80000], 
    30853686643161: [IP_LIGHT[3],   2,   "13",   805*3.8,  20,   0,    240000,    0*80000], # directly
    30853686650497: [IP_LIGHT[3],   7,   "11b",    498.0,  15,   0,    240000,    0*80000], # directly
    30853686650366: [IP_LIGHT[3],   7,   "11a",    498.0,  15,   0,    240000,    0*80000], # directly
}



class CameraNode(Node):
    def __init__(self, name, mac, device_infos):
        self.mac = mac
        self.name = name
        self.timer = 2
        self.device_infos = device_infos
        self.bridge = CvBridge()
        super().__init__(f'camera_{self.name}_node')

        # publishers
        self.save_pub = self.create_publisher(Image, f'save_{self.name}', 1)
        self.view_pub = self.create_publisher(Image, f'view_{self.name}', 10)
        self.inf_pub = self.create_publisher(Image, f'inf_{self.name}', 1)
        self.trigger_pub = self.create_publisher(Bool, f'flash_{self.name}', 10)

        # subscribers
        self.camera_trigger = self.create_subscription(Int16, f'trigger_{self.name}', self.callback, 10)
        self.create_subscription(Float32, f'set_gain_{self.name}', self.set_camera_gain, 1)
        self.create_subscription(Float32, f'set_exp_{self.name}', self.set_camera_exp, 1)

        # service
        self.camera_service = self.create_service(TriggerCamera, f'camtrig_{self.name}', self.service_trigger)
        
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
        nodemap = device.nodemap
        nodemap.read_streamable_node_values_from(FILE_NAME)

        tl_stream_nodemap = device.tl_stream_nodemap
        tl_stream_nodemap["StreamBufferHandlingMode"].value = "NewestOnly"
        tl_stream_nodemap['StreamAutoNegotiatePacketSize'].value = True
        tl_stream_nodemap['StreamPacketResendEnable'].value = True


    def setup_camera_from_list(self, device):
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
        
        nodemap.get_node("PtpEnable").value = True
        # print(nodemap.get_node("PtpStatus"))
        nodemap.get_node("AcquisitionStartMode").value = "PTPSync"
        nodemap.get_node("AcquisitionFrameRate").value = nodemap.get_node("AcquisitionFrameRate").max
        print(nodemap.get_node("AcquisitionFrameRate"))
        # nodemap.get_node("PTPSyncFrameRate").value = 7.0

        nodemap.get_node("GevSCPD").value = BY_MAC[self.mac_add][6]
        nodemap.get_node("GevSCFTD").value = BY_MAC[self.mac_add][7]

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
        # self.setup_camera_from_file(self.device)
        self.setup_camera_from_list(self.device)
        self.device.start_stream(500)


    def stop_camera(self):
        self.device.stop_stream()
        system.destroy_device(self.device)

    
    def callback(self, msg):
        # time1 = time.time()
        if msg.data == 10 or msg.data == 20 or msg.data == 30 or msg.data == 40:
            flash_msg = Bool()
            flash_msg.data = True
            self.trigger_pub.publish(flash_msg)
        # time2 = time.time()
        print(self.timer)
        time.sleep(self.timer)
        # self.timer = self.timer + 0.1
        
        buffer = self.device.get_buffer()
        #time dif
        # print(f"Time in seconds: {time2 - time1}")
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
        if msg.data == 1 or msg.data == 10:
            self.save_pub.publish(image_msg)
        elif msg.data == 2 or msg.data == 20:
            self.view_pub.publish(image_msg)
        elif msg.data == 3 or msg.data == 30:
            self.save_pub.publish(image_msg)
            self.inf_pub.publish(image_msg)
        elif msg.data == 4 or msg.data == 40:
            self.inf_pub.publish(image_msg)
        print(f"Camera: {self.name} frame published at: {msg.data}")
        self.device.requeue_buffer(buffer)

    def service_trigger(self, request, response):
        if request.type == 10 or request.type == 20 or request.type == 30 or request.type == 40:
            flash_msg = Bool()
            flash_msg.data = True
            self.trigger_pub.publish(flash_msg)

        print(self.timer)
        time.sleep(self.timer)
        
        buffer = self.device.get_buffer()

        if self.buffer_bytes_per_pixel is None:
            self.image_width = buffer.width
            self.image_height = buffer.height
            self.buffer_bytes_per_pixel = int(len(buffer.data)/(self.image_width * self.image_height))
        array = (ctypes.c_ubyte * (self.num_channels  * self.image_width * self.image_height)).from_address(ctypes.addressof(buffer.pbytes))
        npndarray = np.ctypeslib.as_array(array).reshape(self.image_height, self.image_width, self.num_channels)
        
        image_msg = self.bridge.cv2_to_imgmsg(npndarray) 
        image_msg.header.stamp = self.get_clock().now().to_msg()

        if image_msg == None:
            response.success = False
            response.status = "No image received"
        elif request.type == 1 or request.type == 10:
            self.save_pub.publish(image_msg)
            response.success = True
            response.status = "Image saved"
        elif request.type == 2 or request.type == 20:
            self.view_pub.publish(image_msg)
            response.success = True
            response.status = "Image shown"
        elif request.type == 3 or request.type == 30:
            self.save_pub.publish(image_msg)
            self.inf_pub.publish(image_msg)
            response.success = True
            response.status = "Image saved and sent to inference"
        elif request.type == 4 or request.type == 40:
            self.inf_pub.publish(image_msg)
            response.success = True
            response.status = "Image sent to inference"
        else:
            response.success = False
            response.status = f"Invalid trigger value: {request.type}"

        self.device.requeue_buffer(buffer)
        return response


    def set_camera_exp(self, msg):
        nodemap = self.device.nodemap
        nodemap.get_node("ExposureAutoUpperLimit").value = msg.data
        print(f'Exposure is set to {nodemap.get_node("ExposureAutoUpperLimit").value}')


    def set_camera_gain(self, msg):
        nodemap = self.device.nodemap
        nodemap.get_node("GainAutoUpperLimit").value = msg.data
        print(f'Gain is set to {nodemap.get_node("GainAutoUpperLimit").value}')


def main(args=None):
    rclpy.init()
    print('... I GOT TO GO HARVEST ...')
    
    all_cameras = ["7a", "7b", "7c", "7d", "7e", "9a", "9b", "9c", "9d", "9e", "1", "3", "4", "10", "11a", "11b", "12", "13"] 
    cameras_to_use = ["11b", "7b", "1"]

    # check for available cameras
    device_infos_no_dup = []
    for device in system.device_infos:
        if device not in device_infos_no_dup:
            device_infos_no_dup.append(device)
    cam_array = []
    executor = MultiThreadedExecutor(num_threads=len(device_infos_no_dup))

    for camera in device_infos_no_dup:
        dec_mac = camera["mac"]
        hex_mac = int(dec_mac.replace(":", ""), 16)
        try:
            camera_name = BY_MAC[hex_mac][2]
            if camera_name in all_cameras:
                camera_node = CameraNode(camera_name, dec_mac, device_infos_no_dup)
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


# def main(args=None):
#     rclpy.init()
#     print('... I GOT TO GO HARVEST ...')
#     # check for available cameras
#     device_infos_no_dup = []
#     for device in system.device_infos:
#         if device not in device_infos_no_dup:
#             device_infos_no_dup.append(device)
#     cam_array = []

#     print(cam_array)

#     for camera in device_infos_no_dup:
#         dec_mac = camera["mac"]
#         hex_mac = int(dec_mac.replace(":", ""), 16)
#         camera_name = BY_MAC[hex_mac][2]
#         if camera_name in all_cameras:
#             camera_node = CameraNode(camera_name, dec_mac, device_infos_no_dup)
#             camera_node.initialize_camera()
#             rclpy.spin(camera_node)

#     rclpy.shutdown()

if __name__ == '__main__':
    main()