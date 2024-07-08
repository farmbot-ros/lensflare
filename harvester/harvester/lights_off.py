#!/usr/bin/python3
import socket
import time
import json
import datetime

class ManimaApiCommunication:
    def __init__(self, udp_ip, udp_port, log_path):
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.log_path = log_path

    def set_parameter_value(self, parameter_name, value):
        if not parameter_name:
            raise ValueError("Parameter name cannot be empty.")

        message = json.dumps({
            "jsonrpc": "2.0",
            "method": "setParameter",
            "params": {"Name": parameter_name, "Value": value},
            "id": 15
        })

        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            try:
                sock.sendto(message.encode("utf-8"), (self.udp_ip, self.udp_port))
                data, server_address = sock.recvfrom(self.udp_port)
                reply_string = data.decode("utf-8")
                return reply_string
                # return "param changed"
            except socket.error as e:
                print(f"Socket error: {e}")
                return None

    def set_scene_duty(self, port_nr, value):
        if not isinstance(port_nr, int) or port_nr < 1 or port_nr > 8:
            raise ValueError("Port number must be an integer between 1 and 8.")
        
        if not isinstance(value, int) or value < 0 or value > 255:
            raise ValueError("Value must be an integer between 0 and 255.")

        parameter_name = f"Port {port_nr} Scene Duty"
        return self.set_parameter_value(parameter_name, value)


    def log_info(self, line_to_add):
        try:
            with open(self.log_path, 'a') as file:
                file.write(line_to_add + '\n')
            print(f"Line added successfully to '{self.log_path}'")
        except FileNotFoundError:
            print(f"Error: File '{self.log_path}' not found.")
        except Exception as e:
            print(f"An error occurred: {e}")

def main():
    sleep = 0.1
    onedev = "192.168.2.252"
    
    devices = ["192.168.2.251", "192.168.2.252", "192.168.2.253"]
    for device in devices:
        controller = ManimaApiCommunication(device, 6512, "/home/bresilla/lights_off.log")
        for i in range(8):
            # print(test.set_scene_duty(i+1, 200))
            # time.sleep(sleep*2)
            print(controller.set_scene_duty(i+1, 0))
            time.sleep(sleep)
        timenow = datetime.datetime.now()
        log_str = "(LIGHTS_OFF) LAST RUN AT:\t" + str(timenow.strftime("%Y_%m_%d_%H_%M"))
        controller.log_info(log_str)


def main2():
    # while(True):
    #     device = "192.168.2.252"
    #     controller = ManimaApiCommunication(device, 6512, "/home/bresilla/lights_off.log")
    #     print(controller.set_scene_duty(6, 255))
    #     time.sleep(1)
    #     print(controller.set_scene_duty(7, 0))
    #     time.sleep(9)

    # device2 = "192.168.2.252"
    # controller2 = ManimaApiCommunication(device2, 6512, "/home/bresilla/lights_off.log")
    # print(controller2.set_scene_duty(6, 250))


if __name__ == "__main__":
    main2()
