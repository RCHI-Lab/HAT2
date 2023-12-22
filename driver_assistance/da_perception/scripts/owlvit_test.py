#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import torch
from PIL import Image
from transformers import OwlViTForObjectDetection, OwlViTProcessor
from transformers.image_utils import ImageFeatureExtractionMixin

model = OwlViTForObjectDetection.from_pretrained("google/owlvit-base-patch32")
processor = OwlViTProcessor.from_pretrained("google/owlvit-base-patch32")

# Use GPU if available
if torch.cuda.is_available():
    device = torch.device("cuda")
else:
    device = torch.device("cpu")

# Set model in evaluation mode
model = model.to(device)
model.eval()

# Sample image
'''
image = Image.open(
    "/home/hat/da_ws/src/driver_assistance/da_perception/images/cup_turn.png"
).convert("RGB")
'''

image = Image.open(
    "/home/hello-akhil/catkin_ws/src/driver_assistance/da_perception/images/cup_turn.png"
).convert("RGB")


# Text queries to search the image for
text_queries = ["red cup", "white cup"]

# Process image and text inputs
inputs = processor(text=text_queries, images=image, return_tensors="pt").to(device)

# Print input names and shapes
# for key, val in inputs.items():
#     print(f"{key}: {val.shape}")

# Get predictions
with torch.no_grad():
    outputs = model(**inputs)

for k, val in outputs.items():
    if k not in {"text_model_output", "vision_model_output"}:
        print(f"{k}: shape of {val.shape}")

print("\nText model outputs")
for k, val in outputs.text_model_output.items():
    print(f"{k}: shape of {val.shape}")

print("\nVision model outputs")
for k, val in outputs.vision_model_output.items():
    print(f"{k}: shape of {val.shape}")

mixin = ImageFeatureExtractionMixin()

# Load example image
image_size = model.config.vision_config.image_size
print(f"image_size: {image_size}")
image = mixin.resize(image, image_size)
input_image = np.asarray(image).astype(np.float32) / 255.0

# Threshold to eliminate low probability predictions
score_threshold = 0.08

# Get prediction logits
logits = torch.max(outputs["logits"][0], dim=-1)
scores = torch.sigmoid(logits.values).cpu().detach().numpy()

# Get prediction labels and boundary boxes
labels = logits.indices.cpu().detach().numpy()
boxes = outputs["pred_boxes"][0].cpu().detach().numpy()


def plot_predictions(input_image, text_queries, scores, boxes, labels):
    fig, ax = plt.subplots(1, 1, figsize=(8, 8))
    ax.imshow(input_image, extent=(0, 1, 1, 0))
    ax.set_axis_off()

    for score, box, label in zip(scores, boxes, labels):
        if score < score_threshold:
            continue

        cx, cy, w, h = box
        ax.plot(
            [cx - w / 2, cx + w / 2, cx + w / 2, cx - w / 2, cx - w / 2],
            [cy - h / 2, cy - h / 2, cy + h / 2, cy + h / 2, cy - h / 2],
            "r",
        )
        ax.text(
            cx - w / 2,
            cy + h / 2 + 0.015,
            f"{text_queries[label]}: {score:1.2f}",
            ha="left",
            va="top",
            color="red",
            bbox={"facecolor": "white", "edgecolor": "red", "boxstyle": "square,pad=.3"},
        )


plot_predictions(input_image, text_queries, scores, boxes, labels)
plt.show()
