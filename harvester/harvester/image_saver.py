import os
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
from rclpy.executors import MultiThreadedExecutor


class ImageSubscriber(Node):
    def __init__(self, base_folder):
        super().__init__('image_saver')
        
        cameras = ["7a", "7b", "7c", "7d", "7e", "9a", "9b", "9c", "9d", "9e", "1", "3", "4", "10", "11a", "11b", "12", "13"] # TODO: use camera names from scan
        for camera in cameras:
            self.create_subscription(Image, f"save_{camera}", lambda msg, cam=camera: self.image_callback(msg, cam), 10)

        self.bridge = CvBridge()
        self.base_folder = base_folder

    def image_callback(self, msg, camera_name):
        print(f"saving image of camera: {camera_name}")
        save_folder = self.base_folder + "/" + self.returns_year_month_day_as_string() + "/" + camera_name
        self.create_folder_if_not_exists(save_folder)
        converted_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        timestamp  = msg.header.stamp
        timestamp = str(timestamp.sec) + str(timestamp.nanosec)
        file_name = os.path.join(save_folder, f"{timestamp}.jpg")
        cv2.imwrite(file_name, converted_image)
        self.get_logger().info(f"Saved image to {file_name}\n")

    def returns_year_month_day_as_string(self):
        import datetime
        now = datetime.datetime.now()
        return now.strftime("%Y%m%d")

    def create_folder_if_not_exists(self, folder_path):
        if not os.path.exists(folder_path):
            print(f"creating path {folder_path}")
            os.makedirs(folder_path)

def main():
    rclpy.init()
    base_folder = '/home/bresilla/images'
    node = ImageSubscriber(base_folder)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
