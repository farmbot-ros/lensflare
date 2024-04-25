import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

from visionplus.msg import Point2D, Mask, BoundingBox2D, DetectionArray, Detection
import cv2
import time

from ultralytics import YOLO
from ultralytics.engine.results import Results
from cv_bridge import CvBridge

class MyNode(Node):
    def __init__(self, args=None):
        super().__init__("apple_segment")
        rclpy.logging.set_logger_level('apple_segment', rclpy.logging.LoggingSeverity.INFO)
        self.declare_parameter("model", "/home/bresilla/ROS_OXBO/best.pt")
        model = self.get_parameter("model").get_parameter_value().string_value
        
        self.device = 'cpu'
               
        self.bridge = CvBridge()
        self.model = YOLO(model, task='segment')
        self.model.to(self.device)
        self.model.fuse()

        self.apple_counter = 0

        self.sub = self.create_subscription(Image, "/oxbo_11a", self.callback, 10)

        self.image_annote_publisher = self.create_publisher(Image, "/oxbo_11b_annptated", 10)
        self.mask_publisher = self.create_publisher(DetectionArray, "/oxbo_11b_mask", 10)
    
    def annotate_image(self, image, mask):
        overlay = image.copy()
        color = (255,0,0)
        cv2.polylines(image, [mask.xy[0].astype(np.int32)], True, color, 2)
        cv2.fillPoly(overlay, [mask.xy[0].astype(np.int32)], color)
        alpha = 0.3
        cv2.addWeighted(overlay, alpha, image, 1 - alpha, 0, image)
        return image

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
        print("inference time apple segmentation: {:.1f} ms and fps: {:.1f}".format((end-start)*1000, fps))

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

            self.apple_counter+=1
            detection.id = str(self.apple_counter)

        # TODO use timestamp of original camera image, not clock
        stamp = self.get_clock().now().to_msg()
        detection_array.header.stamp = stamp
        self.mask_publisher.publish(detection_array)

        # self.image_annote_publisher.publish(self.bridge.cv2_to_imgmsg(annotated_image, "bgr8"))
        self.get_logger().info(f"{self.apple_counter}")
        self.apple_counter = 0
        self.get_logger().info(f"predicted")



def main(args=None):
    rclpy.init(args=args)
    navfix = MyNode(args=args)
    rclpy.spin(navfix)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
