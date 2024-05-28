import numpy as np
import cv2
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from visionplus.msg import Point2D, Mask, BoundingBox2D, DetectionArray, Detection

from ultralytics import YOLO
from ultralytics.engine.results import Results
from cv_bridge import CvBridge

class MyNode(Node):
    def __init__(self, name):
        self.name = name
        super().__init__(f"segment_{self.name}_node")
        rclpy.logging.set_logger_level('pea_segment', rclpy.logging.LoggingSeverity.INFO)
        self.declare_parameter("model", "/home/bresilla/ROS_OXBO/best.pt")
        model = self.get_parameter("model").get_parameter_value().string_value
        
        self.device = 'cuda'
               
        self.bridge = CvBridge()
        self.model = YOLO(model, task='segment')
        self.model.to(self.device)
        self.model.fuse()

        self.pea_counter = 0
        self.pea_percentage = 0

        self.sub = self.create_subscription(Image, f"inf_{self.name}", self.callback, 10)
        self.image_annote_publisher = self.create_publisher(Image, f"annotated_{self.name}", 10)
        self.mask_publisher = self.create_publisher(DetectionArray, f"mask_{self.name}", 10)

    def save_mask_number_csv(self, save_string):
        with open("/home/bresilla/ROS_OXBO/pea_counter.csv", "a") as f:
            f.write(save_string)
    
    def annotate_image(self, image, mask):
        overlay = image.copy()
        color = (255,0,0)
        cv2.polylines(image, [mask.xy[0].astype(np.int32)], True, color, 2)
        cv2.fillPoly(overlay, [mask.xy[0].astype(np.int32)], color)
        alpha = 0.3
        cv2.addWeighted(overlay, alpha, image, 1 - alpha, 0, image)
        return image
    
    def annotated_percentage(self, image, mask):
        if image.shape != mask.shape:
            raise ValueError("Image and mask must have the same shape.")
        total_pixels = image.shape[0] * image.shape[1]
        mask_pixels = np.count_nonzero(mask)
        mask_percentage = (mask_pixels / total_pixels) * 100
        return mask_percentage

    def callback(self, color_msg):
        try:
            original_image = self.bridge.imgmsg_to_cv2(color_msg, "passthrough")
        except Exception as e:
            print(e)
            return

        self.confidence = 0.5
        
        # do the deep learning image analysi
        start = time.time()
        results = self.model.predict(
            source=original_image,
            verbose=False,
            stream=False,
            conf=self.confidence
        )
        end = time.time()
        fps = 1/(end-start)
        print("inference time pea segmentation: {:.1f} ms and fps: {:.1f}".format((end-start)*1000, fps))

        results: Results = results[0].cpu()
        
        if results.masks is None:
            return
        

        detection_array = DetectionArray()
        for yolo_mask, yolo_box in zip(results.masks, results.boxes):
            detection = Detection()
            bbox = BoundingBox2D()
            mask = Mask()
            image_height, image_width = original_image.shape[:2]
            mask.width = image_width
            mask.height = image_height
            for ele in yolo_mask.xy[0].tolist():
                p = Point2D()
                p.x = float(ele[0])
                p.y = float(ele[1])
                mask.data.append(p)
            box = yolo_box.xywh[0]
            bbox.center.position.x = float(box[0])
            bbox.center.position.y = float(box[1])
            bbox.size.x = float(box[2])
            bbox.size.y = float(box[3])
            detection.mask = mask
            detection.bbox = bbox
            detection_array.detections.append(detection)
            annotated_image = self.annotate_image(original_image, yolo_mask)

            # self.pea_percentage += self.annotated_percentage(original_image, yolo_mask)

            self.pea_counter+=1
            detection.id = str(self.pea_counter)

        # TODO use timestamp of original camera image, not clock
        stamp = self.get_clock().now().to_msg()
        unix_timestamp = time.time()
        detection_array.header.stamp = stamp
        self.mask_publisher.publish(detection_array)

        self.save_mask_number_csv(f"{unix_timestamp:.6f},{self.pea_counter},{self.annotated_percentage}\n")

        self.image_annote_publisher.publish(self.bridge.cv2_to_imgmsg(annotated_image, "bgr8"))
        self.get_logger().info(f"{self.pea_counter}")
        self.pea_counter = 0
        self.pea_percentage = 0
        self.get_logger().info(f"predicted")



def main(args=None):
    rclpy.init(args=args)
    navfix = MyNode("11a")
    rclpy.spin(navfix)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
