import argparse
import cv2
import numpy as np


import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

CLASSES = {0: "pea"}
colors = np.random.uniform(0, 255, size=(len(CLASSES), 3))


def draw_bounding_box(img, class_id, confidence, x, y, x_plus_w, y_plus_h):
    label = f'{CLASSES[class_id]} ({confidence:.2f})'
    color = colors[class_id]
    cv2.rectangle(img, (x, y), (x_plus_w, y_plus_h), color, 2)
    cv2.putText(img, label, (x - 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)


def main(onnx_model, input_video):
    model: cv2.dnn.Net = cv2.dnn.readNetFromONNX(onnx_model)
    cap = cv2.VideoCapture(input_video)

    while True:
        ret, original_image = cap.read()
        if not ret:
            break

        height, width, _ = original_image.shape
        left_crop = int(0.2 * width)
        right_crop = int(0.8 * width)
        cropped_image = original_image[:, left_crop:right_crop]

        [height, width, _] = cropped_image.shape
        length = max((height, width))
        image = np.zeros((length, length, 3), np.uint8)
        image[0:height, 0:width] = cropped_image
        scale = length / 512

        blob = cv2.dnn.blobFromImage(image, scalefactor=1 / 255, size=(512, 512), swapRB=True)
        model.setInput(blob)
        outputs = model.forward()

        outputs = np.array([cv2.transpose(outputs[0])])
        rows = outputs.shape[1]

        boxes = []
        scores = []
        class_ids = []

        for i in range(rows):
            classes_scores = outputs[0][i][4:]
            (minScore, maxScore, minClassLoc, (x, maxClassIndex)) = cv2.minMaxLoc(classes_scores)
            if maxScore >= 0.25:
                box = [
                    outputs[0][i][0] - (0.5 * outputs[0][i][2]), outputs[0][i][1] - (0.5 * outputs[0][i][3]),
                    outputs[0][i][2], outputs[0][i][3]]
                boxes.append(box)
                scores.append(maxScore)
                class_ids.append(maxClassIndex)

        result_boxes = cv2.dnn.NMSBoxes(boxes, scores, 0.25, 0.45, 0.5)

        for i in range(len(result_boxes)):
            index = result_boxes[i]
            box = boxes[index]
            draw_bounding_box(cropped_image, class_ids[index], scores[index],
                              round(box[0] * scale), round(box[1] * scale),
                              round((box[0] + box[2]) * scale), round((box[1] + box[3]) * scale))

        cv2.imshow('video', cropped_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


class MyNode(Node):
    def __init__(self, args=None):
        super().__init__("apple_segment")
        self.get_logger().info("Initializing node...")

        self.model: cv2.dnn.Net = cv2.dnn.readNetFromONNX("/home/bresilla/ROS_OXBO/best.onnx")
        self.bridge = CvBridge()

        self.sub = self.create_subscription(Image, "/oxbo_11a", self.callback, 10)
        self.pub = self.create_publisher(Image, "/oxbo_11b_annptated", 10)


    def callback(self, msg):
        self.get_logger().info("Received image")

        image = self.bridge.imgmsg_to_cv2(msg)

        croppedimage = self.detect(image)

        self.pub.publish(self.bridge.cv2_to_imgmsg(croppedimage, "bgr8"))

        print("image_np")


    def detect(self, image):
            height, width, _ = image.shape
            left_crop = int(0.2 * width)
            right_crop = int(0.8 * width)
            cropped_image = image[:, left_crop:right_crop]

            [height, width, _] = cropped_image.shape
            length = max((height, width))
            image = np.zeros((length, length, 3), np.uint8)
            image[0:height, 0:width] = cropped_image
            scale = length / 512

            blob = cv2.dnn.blobFromImage(image, scalefactor=1 / 255, size=(512, 512), swapRB=True)
            self.model.setInput(blob)
            outputs = self.model.forward()

            outputs = np.array([cv2.transpose(outputs[0])])
            rows = outputs.shape[1]

            boxes = []
            scores = []
            class_ids = []

            for i in range(rows):
                classes_scores = outputs[0][i][4:]
                (minScore, maxScore, minClassLoc, (x, maxClassIndex)) = cv2.minMaxLoc(classes_scores)
                if maxScore >= 0.25:
                    box = [
                        outputs[0][i][0] - (0.5 * outputs[0][i][2]), outputs[0][i][1] - (0.5 * outputs[0][i][3]),
                        outputs[0][i][2], outputs[0][i][3]]
                    boxes.append(box)
                    scores.append(maxScore)
                    class_ids.append(maxClassIndex)

            result_boxes = cv2.dnn.NMSBoxes(boxes, scores, 0.25, 0.45, 0.5)

            for i in range(len(result_boxes)):
                index = result_boxes[i]
                box = boxes[index]
                draw_bounding_box(cropped_image, class_ids[index], scores[index],
                                round(box[0] * scale), round(box[1] * scale),
                                round((box[0] + box[2]) * scale), round((box[1] + box[3]) * scale))
            
            return cropped_image



def main(args=None):
    rclpy.init(args=args)
    navfix = MyNode(args=args)
    rclpy.spin(navfix)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
