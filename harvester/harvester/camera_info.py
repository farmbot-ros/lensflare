from arena_api.system import system
from arena_api.buffer import *
from genicam.msg import CameraDevice, CameraDeviceArray
from rclpy.executors import MultiThreadedExecutor


import rclpy
from rclpy.node import Node

IP_LIGHT = {
      0 : None,
      1 : "192.168.2.251", # 7 RIGHT     70:B3:D5:DD:90:FD
      3 : "192.168.2.252", # - CENTER    70:B3:D5:DD:90:EF
      2 : "192.168.2.253", # 9 LEFT      70:B3:D5:DD:90:ED
}

BY_MAC = { 
    ##  MAC             IP      CHANNEL NAME       
    30853686642969: [IP_LIGHT[2],   7,   "9b"],
    30853686643065: [IP_LIGHT[2],   1,   "9a"],
    30853686646563: [IP_LIGHT[2],   2,   "9c"],
    30853686653149: [IP_LIGHT[2],   5,   "9d"],
    30853686643056: [IP_LIGHT[2],   6,   "9e"],
    30853686646554: [IP_LIGHT[1],   6,   "7a"],
    30853686652294: [IP_LIGHT[1],   1,   "7b"],
    30853686445113: [IP_LIGHT[1],   5,   "7c"],
    30853686646528: [IP_LIGHT[1],   2,   "7d"],
    30853686653140: [IP_LIGHT[1],   3,   "7e"],
    30853686643187: [IP_LIGHT[0],   0,   "1"],
    30853686650340: [IP_LIGHT[3],   6,   "4"],
    30853686646397: [IP_LIGHT[3],   5,   "3"],
    30853686643161: [IP_LIGHT[3],   2,   "13"],
    30853686643152: [IP_LIGHT[3],   1,   "12"],
    30853686646406: [IP_LIGHT[3],   3,   "10"],
    30853686650497: [IP_LIGHT[3],   7,   "11a"],
    30853686650366: [IP_LIGHT[3],   7,   "11b"],
}

camera_info = None

class CameraArray(Node):
    def __init__(self):
        super().__init__('camera_array')
        self.publish_array_info_pub = self.create_timer(0.001, self.publish_array_info)
        self.array_pub = self.create_publisher(CameraDeviceArray, '/caminfo', 10)

    def publish_array_info(self):
        global camera_info
        if camera_info is not None:
            self.array_pub.publish(camera_info)
            ids = [camera.id for camera in camera_info.cameras]
            # self.get_logger().info(f"cameras infos of {ids} are published")

class CameraInfo(Node):
    def __init__(self):
        super().__init__('camera_info')
        self.update_camera_info_pub = self.create_timer(30, self.update_camera_info)
        self.update_camera_info()

    """
    A list of dictionaries used to create devices. Each dictionary represents a discovered device on the network.
    A device info dictionary has the following keys: `model`,`vendor`, `serial`, `ip`, `subnetmask`,
    `defaultgateway`, `mac`, `name`, `dhcp`, `presistentip`, `lla`, and `version`. 
    """
    
    def update_camera_info(self):
        global camera_info
        device_infos_no_dup = []
        for device in system.device_infos:
            if device not in device_infos_no_dup:
                device_infos_no_dup.append(device)
        camera_array = CameraDeviceArray()
        for device in device_infos_no_dup:
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

            hex_mac = int(device["mac"].replace(":", ""), 16)
            if hex_mac in BY_MAC:
                camera_device.light_ip = BY_MAC[hex_mac][0]
                camera_device.light_port = "6512"
                camera_device.light_channel = str(BY_MAC[hex_mac][1])
                camera_device.id = BY_MAC[hex_mac][2]

            camera_array.cameras.append(camera_device)
        camera_info = camera_array
        self.get_logger().info(f"camera info is updated, {len(camera_array.cameras)} cameras are found")
        

def main(args=None):
    rclpy.init()
    print('... GETTING CAMERAS ...')
    executor = MultiThreadedExecutor(num_threads=2)
    camera_info_node = CameraInfo()
    camera_array_node = CameraArray()
    try:
        executor.add_node(camera_info_node)
        executor.add_node(camera_array_node)
        try:
            executor.spin()
        except Exception as e:
            print(f"An error occurred: {str(e)}")
    except Exception as e:
        print(f"An error occurred: {str(e)}")
    finally:
        executor.remove_node(camera_info_node)
        executor.remove_node(camera_array_node)
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()