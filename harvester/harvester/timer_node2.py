import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int16
import time


class TimerNode(Node):
    def __init__(self, camera_array, time_interval, group, message):
        super().__init__(f'timer_node_{str(group)}')
        self.trigger_pub_dict = {}
        self.message = message
        self.camera_array = camera_array
        self.inference_timer = self.create_timer(time_interval, self.timer_callback)
        for camera in self.camera_array:
            print(camera)
            self.trigger_pub_dict[camera] = self.create_publisher(Int16, f"trigger_{camera}", 10)


    def timer_callback(self):
        for camera in self.camera_array:
            trigger_msg = Int16()
            trigger_msg.data = self.message
            self.trigger_pub_dict[camera].publish(trigger_msg)
            print(f"hi from {camera}")
            time.sleep(0.1)


def main(args=None):
    rclpy.init()
    print('... TRIGGERING CAMERAS ...')

    inference_cameras = ["11a", "11b", "13"]
    acquisition_cameras = ["7a", "7b", "7c", "7d", "7e", "9a", "9b", "9c", "9d", "9e", "1", "3", "4", "10", "12"]

    executor = MultiThreadedExecutor(num_threads=2)
    camera_group_1 = TimerNode(inference_cameras, 6, "inference", 1)
    camera_group_2 = TimerNode(acquisition_cameras, 18, "aquisition", 1)
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
