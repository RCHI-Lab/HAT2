#!/home/hat/da_ws/src/driver_assistance/da_perception/.venv/bin/python3

from __future__ import annotations

import cv2
import numpy as np
import rospy
import torch
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from transformers import OwlViTForObjectDetection, OwlViTProcessor


class OwlViTRos:
    def __init__(self):
        self.bridge = CvBridge()
        self.latest_image: np.ndarray | None = None
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        self.model = OwlViTForObjectDetection.from_pretrained("google/owlvit-base-patch32")
        self.processor = OwlViTProcessor.from_pretrained("google/owlvit-base-patch32")
        # Use GPU if available
        if torch.cuda.is_available():
            self.device = torch.device("cuda")
        else:
            self.device = torch.device("cpu")
        self.model = self.model.to(self.device)
        self.model.eval()

    def callback(self, msg: Image):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

    def process(self):
        if self.latest_image is None:
            return
        queries = ["red cup", "blue cup", "green cup", "pink cup"]
        cv2_img = self.latest_image.copy()
        cv2_img = cv2.rotate(cv2_img, cv2.ROTATE_90_CLOCKWISE)
        cv2_img = self.center_crop_to_square(cv2_img)

        # input image need to be rgb
        rgb_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2RGB)
        inputs = self.processor(text=queries, images=rgb_img, return_tensors="pt").to(self.device)
        with torch.no_grad():
            outputs = self.model(**inputs)

        # Threshold to eliminate low probability predictions
        score_threshold = 0.08
        # Get prediction logits
        logits = torch.max(outputs["logits"][0], dim=-1)
        scores = torch.sigmoid(logits.values).cpu().detach().numpy()
        # Get prediction labels and boundary boxes
        labels = logits.indices.cpu().detach().numpy()
        boxes = outputs["pred_boxes"][0].cpu().detach().numpy()

        for score, box, label in zip(scores, boxes, labels):
            if score < score_threshold:
                continue
            self.draw_detection(
                cv2_img,
                {
                    "label": queries[label],
                    "confidence": score,
                    "box": box,
                },
            )
        cv2.imshow("window", cv2_img)
        cv2.waitKey(3)

    def center_crop_to_square(self, image):
        height, width = image.shape[:2]

        # Calculate the side length of the square
        side_length = min(height, width)

        # Calculate the coordinates for cropping
        y1 = (height - side_length) // 2
        y2 = y1 + side_length
        x1 = (width - side_length) // 2
        x2 = x1 + side_length

        return image[y1:y2, x1:x2]

    def draw_detection(self, image, detection_dict):
        font_scale = 0.75
        line_color = [0, 0, 0]
        line_width = 1
        font = cv2.FONT_HERSHEY_PLAIN
        class_label = detection_dict["label"]
        confidence = detection_dict["confidence"]
        box = detection_dict["box"]
        original_height, original_width, num_color = image.shape
        cx, cy, w, h = box
        x_min = int((cx - w / 2) * original_width)
        x_max = int((cx + w / 2) * original_width)
        y_min = int((cy - h / 2) * original_height)
        y_max = int((cy + h / 2) * original_height)
        print(x_min, y_min, x_max, y_max)
        # x_min, y_min, x_max, y_max = box
        output_string = "{0}, {1:.2f}".format(class_label, confidence)
        color = (0, 0, 255)
        rectangle_line_thickness = 2  # 1
        cv2.rectangle(image, (x_min, y_min), (x_max, y_max), color, rectangle_line_thickness)

        # see the following page for a helpful reference
        # https://stackoverflow.com/questions/51285616/opencvs-gettextsize-and-puttext-return-wrong-size-and-chop-letters-with-low

        label_background_border = 2
        (label_width, label_height), baseline = cv2.getTextSize(
            output_string, font, font_scale, line_width
        )
        label_x_min = x_min
        label_y_min = y_min
        label_x_max = x_min + (label_width + (2 * label_background_border))
        label_y_max = y_min + (label_height + baseline + (2 * label_background_border))

        text_x = label_x_min + label_background_border
        text_y = (label_y_min + label_height) + label_background_border

        cv2.rectangle(
            image,
            (label_x_min, label_y_min),
            (label_x_max, label_y_max),
            (255, 255, 255),
            cv2.FILLED,
        )
        cv2.putText(
            image,
            output_string,
            (text_x, text_y),
            font,
            font_scale,
            line_color,
            line_width,
            cv2.LINE_AA,
        )


if __name__ == "__main__":
    rospy.init_node("OwlViT", anonymous=True)
    Owl = OwlViTRos()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        Owl.process()
        rate.sleep()

    cv2.destroyAllWindows()
