from groundingdino.util.inference import load_model, load_image, predict, annotate
import cv2
from time import perf_counter

model = load_model(
    ".venv/lib/python3.8/site-packages/groundingdino/config/GroundingDINO_SwinT_OGC.py",
    "weights/groundingdino_swint_ogc.pth",
)
IMAGE_PATH = "images/cup_turn_noresize.png"
TEXT_PROMPT = "cup"
BOX_TRESHOLD = 0.35
TEXT_TRESHOLD = 0.25

image_source, image = load_image(IMAGE_PATH)

t1_start = perf_counter()

for _ in range(5):
    boxes, logits, phrases = predict(
        model=model,
        image=image,
        caption=TEXT_PROMPT,
        box_threshold=BOX_TRESHOLD,
        text_threshold=TEXT_TRESHOLD,
        device="cpu",
    )

t1_stop = perf_counter()

print("Elapsed time:", t1_stop, t1_start)
print("Elapsed time during the whole program in seconds:", t1_stop - t1_start)

annotated_frame = annotate(image_source=image_source, boxes=boxes, logits=logits, phrases=phrases)
cv2.imwrite("annotated_image.jpg", annotated_frame)
