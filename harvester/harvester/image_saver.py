import os
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
from rclpy.executors import MultiThreadedExecutor


class ImageSubscriber(Node):
    def __init__(self, topic_name, base_folder, timer_period=1.0):
        super().__init__('image_subscriber')
        self.the_image = None
        self.subscription = self.create_subscription(Image, topic_name, self.image_callback, 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.base_folder = base_folder + "/" + self.returns_year_month_day_as_string() + "/" + topic_name
        self.create_folder_if_not_exists(self.base_folder)
        # print(self.returns_year_month_day_as_string())

    def image_callback(self, msg):
        try:
            self.the_image = msg
        except Exception as e:
            self.get_logger().error("Error converting ROS Image to OpenCV image: %s" % str(e))
            return

    def timer_callback(self):
        if self.the_image is None: return
        converted_image = self.bridge.imgmsg_to_cv2(self.the_image, "passthrough")
        timestamp  = self.the_image.header.stamp
        # print(f"Received image with timestamp {timestamp}")
        timestamp = str(timestamp.sec) + str(timestamp.nanosec)
        file_name = os.path.join(self.base_folder, f"{timestamp}.jpg")
        cv2.imwrite(file_name, converted_image)
        self.get_logger().info(f"Saved image to {file_name}")

    def returns_year_month_day_as_string(self):
        import datetime
        now = datetime.datetime.now()
        return now.strftime("%Y%m%d")

    def create_folder_if_not_exists(self, folder_path):
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)

# def main():
#     rclpy.init()
#     topic_name = 'oxbo_test'
#     base_folder = '/mnt/data'
#     node = ImageSubscriber(topic_name, base_folder)
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()

def main(args=None):
    rclpy.init()
    print('Hi from saver.')
    base_folder = '/mnt/data'
    timer_period = 60.0
    try:
        camera_11a_node = ImageSubscriber('oxbo_11a', base_folder, timer_period)
        camera_11b_node = ImageSubscriber('oxbo_11b', base_folder, timer_period)

        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(camera_11a_node)
        executor.add_node(camera_11b_node)
        try:
            executor.spin()
        finally:
            executor.shutdown()

    except Exception as e:
        print(f"An error occurred: {str(e)}")

    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
