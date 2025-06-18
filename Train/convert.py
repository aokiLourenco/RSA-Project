import os
import json
import shutil
import random
import cv2

# Configuration
IMAGE_DIR = "images_raw"
JSON_DIR = "annotations_json"
OUTPUT_DIR = "dataset"
TRAIN_SPLIT = 0.8

# Create output directories
for split in ["train", "val"]:
    os.makedirs(os.path.join(OUTPUT_DIR, "images", split), exist_ok=True)
    os.makedirs(os.path.join(OUTPUT_DIR, "labels", split), exist_ok=True)

# Collect all image files
image_files = [f for f in os.listdir(IMAGE_DIR) if f.lower().endswith((".jpg", ".png"))]
random.shuffle(image_files)

split_index = int(len(image_files) * TRAIN_SPLIT)
train_files = image_files[:split_index]
val_files = image_files[split_index:]

def convert_annotation(image_file, split):
    json_file = image_file.rsplit(".", 1)[0] + ".json"
    image_path = os.path.join(IMAGE_DIR, image_file)
    json_path = os.path.join(JSON_DIR, json_file)

    # Load image
    image = cv2.imread(image_path)
    if image is None:
        print(f"Skipping invalid image: {image_path}")
        return

    height, width = image.shape[:2]

    # Load JSON
    try:
        with open(json_path, "r") as f:
            data = json.load(f)
    except Exception as e:
        print(f"Error reading {json_path}: {e}")
        return

    label_lines = []
    for label in data.get("labels", []):
        if label["name"] == "Vacant":
            x1, y1, x2, y2 = label["x1"], label["y1"], label["x2"], label["y2"]
            x_center = ((x1 + x2) / 2) / width
            y_center = ((y1 + y2) / 2) / height
            box_width = (x2 - x1) / width
            box_height = (y2 - y1) / height

            label_lines.append(f"0 {x_center:.6f} {y_center:.6f} {box_width:.6f} {box_height:.6f}")
        elif label["name"] == "Occupied":
            continue
            # x1, y1, x2, y2 = label["x1"], label["y1"], label["x2"], label["y2"]
            # x_center = ((x1 + x2) / 2) / width
            # y_center = ((y1 + y2) / 2) / height
            # box_width = (x2 - x1) / width
            # box_height = (y2 - y1) / height

            # label_lines.append(f"0 {x_center:.6f} {y_center:.6f} {box_width:.6f} {box_height:.6f}")

    # Save label file
    label_output_path = os.path.join(OUTPUT_DIR, "labels", split, image_file.rsplit(".", 1)[0] + ".txt")
    with open(label_output_path, "w") as f:
        f.write("\n".join(label_lines))

    # Copy image to split folder
    shutil.copy(image_path, os.path.join(OUTPUT_DIR, "images", split, image_file))

# Process files
for image_file in train_files:
    convert_annotation(image_file, "train")

for image_file in val_files:
    convert_annotation(image_file, "val")

# Write data.yaml
yaml_path = os.path.join(OUTPUT_DIR, "data.yaml")
with open(yaml_path, "w") as f:
    f.write(
        f"train: {os.path.abspath(os.path.join(OUTPUT_DIR, 'images/train'))}\n"
        f"val: {os.path.abspath(os.path.join(OUTPUT_DIR, 'images/val'))}\n\n"
        f"nc: 1\n"
        f"names: ['Vacant']\n"
    )

print("Dataset prepared in YOLO format.")
