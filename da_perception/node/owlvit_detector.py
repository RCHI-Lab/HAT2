import cv2
import torch
from transformers import OwlViTForObjectDetection, OwlViTProcessor


class OwlViTObjectDetector:
    def __init__(self, queries=None, score_threshold=0.08):
        if queries is None:
            queries = ["red cup", "blue cup", "green cup", "pink cup"]
        self.queries = queries
        self.score_threshold = score_threshold
        self.model = OwlViTForObjectDetection.from_pretrained("google/owlvit-base-patch32")
        self.processor = OwlViTProcessor.from_pretrained("google/owlvit-base-patch32")
        # Use GPU if available
        if torch.cuda.is_available():
            self.device = torch.device("cuda")
        else:
            self.device = torch.device("cpu")
        self.model = self.model.to(self.device)
        self.model.eval()

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

    def apply_to_image(self, bgr_img, draw_output=False, crop=False):
        original_height, original_width, num_color = bgr_img.shape
        if crop:
            bgr_img = self.center_crop_to_square(bgr_img)
        rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)
        new_height, new_width, num_color = rgb_img.shape

        inputs = self.processor(text=self.queries, images=rgb_img, return_tensors="pt").to(
            self.device
        )
        with torch.no_grad():
            outputs = self.model(**inputs)

        # Get prediction logits
        logits = torch.max(outputs["logits"][0], dim=-1)
        scores = torch.sigmoid(logits.values).cpu().detach().numpy()
        # Get prediction labels and boundary boxes
        labels = logits.indices.cpu().detach().numpy()
        boxes = outputs["pred_boxes"][0].cpu().detach().numpy()

        def bound_x(x_in):
            x_out = max(x_in, 0)
            x_out = min(x_out, original_width - 1)
            return x_out

        def bound_y(y_in):
            y_out = max(y_in, 0)
            y_out = min(y_out, original_height - 1)
            return y_out

        results = []
        idx = 0
        for score, box, label in zip(scores, boxes, labels):
            if score < self.score_threshold:
                continue

            # collect and prepare detected objects
            box_center_x, box_center_y, box_width, box_height = box

            x_min = (box_center_x - (box_width / 2.0)) * new_width
            y_min = (box_center_y - (box_height / 2.0)) * new_height
            x_max = x_min + (box_width * new_width)
            y_max = y_min + (box_height * new_height)

            if crop:
                y_min += abs(original_height - original_width) // 2
                y_max += abs(original_height - original_width) // 2

            x_min = bound_x(int(round(x_min)))
            y_min = bound_y(int(round(y_min)))
            x_max = bound_x(int(round(x_max)))
            y_max = bound_y(int(round(y_max)))

            corner_box = (x_min, y_min, x_max, y_max)

            results.append(
                {
                    "id": idx,
                    "class_id": label,
                    "label": self.queries[label],
                    "confidence": score,
                    "box": corner_box,
                }
            )
            idx += 1

        output_image = None
        if draw_output:
            output_image = bgr_img.copy()
            for detection_dict in results:
                self.draw_detection(output_image, detection_dict)

        return results, output_image

    def draw_detection(self, image, detection_dict):
        font_scale = 0.75
        line_color = [0, 0, 0]
        line_width = 1
        font = cv2.FONT_HERSHEY_PLAIN
        class_label = detection_dict["label"]
        confidence = detection_dict["confidence"]
        box = detection_dict["box"]
        x_min, y_min, x_max, y_max = box
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
