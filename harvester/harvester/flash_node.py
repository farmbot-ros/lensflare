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

from harvester_interfaces.srv import TriggerFlash
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import Int16, Bool



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
    30853686652294: [IP_LIGHT[2],   1,   "7b",   521*EXP,  30],
    30853686445113: [IP_LIGHT[1],   5,   "7c",   521*EXP,  30],
    30853686646528: [IP_LIGHT[1],   2,   "7d",   521*EXP,  30],
    30853686653140: [IP_LIGHT[1],   3,   "7e",   521*EXP,  30],
    30853686643187: [IP_LIGHT[2],   1,   "1",   2847*1.1,  20],
    # 30853686643187: [IP_LIGHT[0],   0,   "1",   2847*1.1,  20],
    30853686650340: [IP_LIGHT[3],   6,   "4",    528*2.8,  30],
    30853686646397: [IP_LIGHT[3],   5,   "3",    810*2.8,  20],
    30853686643161: [IP_LIGHT[3],   2,   "13",   805*3.8,  20],
    30853686643152: [IP_LIGHT[3],   1,   "12",   460*3.8,  40],
    30853686646406: [IP_LIGHT[3],   3,   "10",     867.0,  30],
    30853686650497: [IP_LIGHT[3],   7,   "11a",    498.0,  15],
    30853686650366: [IP_LIGHT[2],   1,   "11b",    498.0,  15],
    # 30853686650366: [IP_LIGHT[3],   7,   "11b",    498.0,  15],
}

class FlashNode(Node):
    def __init__(self, udp_port=6512):
        self.udp_port = udp_port
        super().__init__(f'flash_node')


        for mac, light_info in BY_MAC.items():
            ip, channel, name = light_info[:3]
            self.create_subscription(
                Bool,
                f"flash_{name}",
                lambda msg, ip=ip, channel=channel: self.flash_callback(msg, ip, channel),
                10,
            )
        
         # service
        self.camera_service = self.create_service(TriggerFlash, f'flash_trigger', self.service_trigger)


        
    def flash_callback(self, msg, ip, channel):
        if msg.data:
            print(f"light controller ip: {ip}, channel number: {channel}")
            for i in range(6):
                self.pulse(ip, channel)
                time.sleep(0.3)



    def service_trigger(self, request, response):
        response.success = True
        mac = request.mac_address
        if mac in BY_MAC:
            ip, channel, name = BY_MAC[mac][:3]
            for i in range(6):
                # self.pulse(ip, channel)
                print(f"fflashing {name}...")
                time.sleep(0.3)
        else:
            response.success = False
            response.message = "No light controller found with this MAC address."
        return response



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


def main(args=None):
    rclpy.init()
    print('... LETS GO FLASH SOME LIGHTS ...')

    flash_node = FlashNode()
    try:
        rclpy.spin(flash_node)
    except KeyboardInterrupt:
        pass
    finally:
        flash_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()