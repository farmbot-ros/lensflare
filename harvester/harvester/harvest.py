import time
import datetime
from arena_api.enums import PixelFormat
from arena_api.__future__.save import Writer
from arena_api.system import system
from arena_api.buffer import BufferFactory
import cv2
import ctypes
import numpy as np
import socket
import time
import json
import os

#
BUFFER_NUMBER = 30

IP_LIGHT = {
      0 : None,
      1 : "192.168.1.139",
      2 : "192.168.1.124",
      3 : "192.168.1.125",
}

BY_MAC = { 
    30853686642969: [IP_LIGHT[2], 7, "9b"],
    30853686643065: [IP_LIGHT[2], 1, "9a"],
    30853686646563: [IP_LIGHT[2], 2, "9c"],
    30853686653149: [IP_LIGHT[2], 5, "9d"],
    30853686643056: [IP_LIGHT[2], 6, "9e"],
    30853686646554: [IP_LIGHT[1], 6, "7a"],
    30853686652294: [IP_LIGHT[1], 1, "7b"],
    30853686445113: [IP_LIGHT[1], 5, "7c"],
    30853686646528: [IP_LIGHT[1], 2, "7d"],
    30853686653140: [IP_LIGHT[1], 3, "7e"],
    30853686643187: [IP_LIGHT[0], 0, "1"],
    30853686650340: [IP_LIGHT[3], 6, "4"],
    30853686646397: [IP_LIGHT[3], 5, "3"],
    30853686643161: [IP_LIGHT[3], 2, "13"],
    30853686643152: [IP_LIGHT[3], 1, "12"],
    30853686646406: [IP_LIGHT[3], 3, "10"],
    30853686650497: [IP_LIGHT[3], 7, "11a"],
    30853686650366: [IP_LIGHT[3], 7, "11b"],
}

BY_NAME = { 
    "9b":   [30853686642969, 2, 7],
    "9a":   [30853686643065, 2, 1],
    "9c":   [30853686646563, 2, 2],
    "9d":   [30853686653149, 2, 5],
    "9e":   [30853686643056, 2, 6],
    "7a":   [30853686646554, 1, 6],
    "7b":   [30853686652294, 1, 1],
    "7c":   [30853686445113, 1, 5],
    "7d":   [30853686646528, 1, 2],
    "7e":   [30853686653140, 1, 3],
    "1":    [30853686643187, 3, 3],
    "4":    [30853686650340, 3, 6],
    "3":    [30853686646397, 3, 5],
    "13":   [30853686643161, 3, 2],
    "12":   [30853686643152, 3, 1],
    "10":   [30853686646406, 2, 3],
    "11a":  [30853686650497, 3, 7],
    "11b":  [30853686650366, 3, 7],
}


class LightController:
    def __init__(self, udp_port):
        self.udp_port = udp_port

    def set_parameter_value(self, udp_ip, parameter_name, value):
        if not parameter_name: raise ValueError("Parameter name cannot be empty.")
        message = json.dumps({"jsonrpc": "2.0", "method": "setParameter", "params": {"Name": parameter_name, "Value": value}, "id": 15})
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            try:
                sock.sendto(message.encode("utf-8"), (udp_ip, self.udp_port))
                data, server_address = sock.recvfrom(4096)
                reply_string = data.decode("utf-8")
                return reply_string
            except socket.error as e:
                print(f"Socket error: {e}")
                return None

    def duty(self, udp_ip, port_nr, value):
        if not isinstance(port_nr, int) or port_nr < 1 or port_nr > 8: 
            raise ValueError("Port number must be an integer between 1 and 8.")
        if not isinstance(value, int) or value < 0 or value > 255: 
            raise ValueError("Value must be an integer between 0 and 255.")
        parameter_name = f"Port {port_nr} Scene Duty"
        return self.set_parameter_value(udp_ip, parameter_name, value)


class CameraHarverster:
    def __init__(self, data_folder: str):
        self.data_folder = data_folder
        if not os.path.exists(self.data_folder): 
            os.makedirs(self.data_folder)
        self.tries = 10
        self.sleep_time_secs = 10
        self.devices = self.initialize_devices()
        for device in self.devices:
            self.node_setup(device)
            self.stream_setup(device)

    def initialize_devices(self):
        tries = 0
        devices = None
        while tries < self.tries:
            devices = system.create_device()
            if not devices:
                print(
                    f'Try {tries+1} of {self.tries}: waiting for {self.sleep_time_secs} '
                    f'secs for a device to be connected!')
                for sec_count in range(self.sleep_time_secs):
                    time.sleep(1)
                    print(f'{sec_count + 1 } seconds passed ',
                        '.' * sec_count, end='\r')
                tries += 1
            else:
                print(len(devices))
                return devices
        else:
            raise Exception(f'No device found! Please connect a device and run the example again.')

    def node_setup(self, device):
        nodemap = device.nodemap
        nodes = nodemap.get_node(['Width', 'Height', 'PixelFormat', 'GevMACAddress'])
        nodes['Width'].value = nodes['Width'].max
        nodes['Height'].value = nodes['Height'].max
        nodes['PixelFormat'].value = 'BGR8'
        mac_add = nodes['GevMACAddress'].value
        folder_name = BY_MAC[mac_add][2]
        if not os.path.exists(self.data_folder + "/" + folder_name):
            os.makedirs(self.data_folder + "/" + folder_name)

    def stream_setup(self, device):
        tl_stream_nodemap = device.tl_stream_nodemap
        tl_stream_nodemap["StreamBufferHandlingMode"].value = "NewestOnly"
        tl_stream_nodemap['StreamAutoNegotiatePacketSize'].value = True
        tl_stream_nodemap['StreamPacketResendEnable'].value = True

    def save(self, buffer, folder_name: str, file_name: str):
        converted = BufferFactory.convert(buffer, PixelFormat.BGR8)
        print(f"Converted image to {PixelFormat.BGR8.name}")
        print(f'Prepare Image Writer')
        writer = Writer()
        writer.pattern = f'{self.data_folder}/{folder_name}/image_{file_name}.jpg'
        writer.save(converted)
        print(f'Image saved')
        BufferFactory.destroy(converted)

    def view(self, buffer, window_title: str = "WINDOW"):
        item = BufferFactory.copy(buffer)
        buffer_bytes_per_pixel = int(len(item.data)/(item.width * item.height))
        array = (ctypes.c_ubyte * 3 * item.width * item.height).from_address(ctypes.addressof(item.pbytes))
        npndarray = np.ndarray(buffer=array, dtype=np.uint8, shape=(item.height, item.width, buffer_bytes_per_pixel))
        cv2.imshow(window_title, npndarray)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        BufferFactory.destroy(item)

lights = True
if __name__ == "__main__":
    print("Starting...\n")
    timenow = datetime.datetime.now()
    base_folder = "/mnt/data/"
    folder_name = base_folder + str(timenow.strftime("%Y_%m_%d"))
    if lights:
        ctrl = LightController(6512)
    harvester = CameraHarverster(folder_name)
    looper = 0
    while True:
        looper += 1
        for device in harvester.devices:
            stamp = time.time()
            mac_add = device.nodemap['GevMACAddress'].value
            print(f"Capturing image {str(looper).zfill(5)} from {mac_add} at {stamp}")
            ip_address = BY_MAC[mac_add][0]
            light_channel = BY_MAC[mac_add][1]
            if ip_address is not None and lights:
                print(ctrl.duty(ip_address, light_channel, 250))
            try:
                device.start_stream(BUFFER_NUMBER)
            except Exception as e:
                if ip_address is not None and lights: 
                    print(ctrl.duty(ip_address, light_channel, 0))
                print(f"Error: {e}")
                continue
            buffers = device.get_buffer(BUFFER_NUMBER)
            for count, buffer in enumerate(buffers):
                if count > int(BUFFER_NUMBER/2):
                    harvester.save(
                        buffer, 
                        BY_MAC[mac_add][2], f"{str(stamp)}___{str(looper).zfill(5)}__{str(count)}"
                    )
            device.requeue_buffer(buffers)
            device.stop_stream()
            if ip_address is not None and lights: 
                print(ctrl.duty(ip_address, light_channel, 0))
    # print("\Completed")