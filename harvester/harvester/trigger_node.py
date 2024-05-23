import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int16, Bool
import time


class TriggerNode(Node):
    def __init__(self, camera_array, group, message, sequence_interval, inter_camera_delay):
        super().__init__(f'trigger_node_{str(group)}')
        self.trigger_pub_dict = {}
        self.flash_pub_dict = {}
        self.message = message
        self.camera_array = camera_array
        self.inter_camera_delay = inter_camera_delay
        self.create_timer(sequence_interval, self.timer_callback)
        for camera in self.camera_array:
            self.trigger_pub_dict[camera] = self.create_publisher(Int16, f"trigger_{camera}", 10)
            self.flash_pub_dict[camera] = self.create_publisher(Bool, f"flash_{camera}",10)


    def timer_callback(self):
        for camera in self.camera_array:
            flash_msg = Bool()
            flash_msg.data = True
            trigger_msg = Int16()
            trigger_msg.data = self.message
            self.flash_pub_dict[camera].publish(flash_msg)
            time.sleep(0.01) # TODO: tweak this delay such that it fits capturing the image which lights
            self.trigger_pub_dict[camera].publish(trigger_msg)
            print(f"hi from {camera}")
            time.sleep(self.inter_camera_delay)


def main(args=None):
    rclpy.init()
    print('... TRIGGERING CAMERAS ...')

    inference_cameras = ["11a", "11b", "13"]
    acquisition_cameras = ["7a", "7b", "7c", "7d", "7e", "9a", "9b", "9c", "9d", "9e", "1", "3", "4", "10", "12"]

    executor = MultiThreadedExecutor(num_threads=2)
    camera_group_1 = TriggerNode(inference_cameras, "inference", 1, sequence_interval=10, inter_camera_delay=1)
    camera_group_2 = TriggerNode(acquisition_cameras, "aquisition", 1, sequence_interval=60, inter_camera_delay=1)
    try:
        executor.add_node(camera_group_1)
        executor.add_node(camera_group_2)
        try:
            executor.spin()
        except Exception as e:
            print(f"An error occurred: {str(e)}")
    except Exception as e:
        print(f"An error occurred: {str(e)}")
    finally:
        executor.remove_node(camera_group_1)
        executor.remove_node(camera_group_2)
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
