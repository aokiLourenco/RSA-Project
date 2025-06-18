import os
import cv2
import albumentations as A
import numpy as np

# Paths train
IMAGE_DIR = "frames/images/train"
LABEL_DIR = "frames/labels/train"
OUTPUT_IMAGE_DIR = "augmented_dataset/tilted/images/train"
OUTPUT_LABEL_DIR = "augmented_dataset/tilted/labels/train"

# Paths val
# IMAGE_DIR = "dataset/images/val"
# LABEL_DIR = "dataset/labels/val"
# OUTPUT_IMAGE_DIR = "augmented_dataset/images/val"
# OUTPUT_LABEL_DIR = "augmented_dataset/labels/val"

# Vacant paths train

# IMAGE_DIR = "vacant/train/images"
# LABEL_DIR = "vacant/train/labels"
# OUTPUT_IMAGE_DIR = "augmented_dataset/vacant/train/images"
# OUTPUT_LABEL_DIR = "augmented_dataset/vacant/train/labels"

# Vacant paths val
# IMAGE_DIR = "vacant/val/images"
# LABEL_DIR = "vacant/val/labels"
# OUTPUT_IMAGE_DIR = "augmented_dataset/vacant/val/images"
# OUTPUT_LABEL_DIR = "augmented_dataset/vacant/val/labels"


# Own paths train
# IMAGE_DIR = "own_train/images/train"
# LABEL_DIR = "own_train/labels/train"
# OUTPUT_IMAGE_DIR = "augmented_dataset_own/images/train"
# OUTPUT_LABEL_DIR = "augmented_dataset_own/labels/train"


# Create output directories
os.makedirs(OUTPUT_IMAGE_DIR, exist_ok=True)
os.makedirs(OUTPUT_LABEL_DIR, exist_ok=True)

# Define augmentations
transform = A.Compose([
    A.Rotate(limit=90, p=1, border_mode=cv2.BORDER_CONSTANT),
], bbox_params=A.BboxParams(
        format='yolo',
        clip=True,
    ))


def load_labels(label_path):
    with open(label_path, 'r') as f:
        lines = f.readlines()
    boxes = []
    classes = []
    for line in lines:
        parts = line.strip().split()
        classes.append(int(parts[0]))
        boxes.append(list(map(float, parts[1:])))
    return boxes, classes

def save_labels(label_path, boxes, classes):
    with open(label_path, 'w') as f:
        for cls, box in zip(classes, boxes):
            f.write(f"{cls} {' '.join(map(str, box))}\n")

# Number of augmentations per image
AUGMENTATIONS_PER_IMAGE = 3

# Apply augmentation
for filename in os.listdir(IMAGE_DIR):
    if not filename.endswith((".jpg", ".png", ".jpeg")):
        continue

    image_path = os.path.join(IMAGE_DIR, filename)
    label_path = os.path.join(LABEL_DIR, filename.rsplit(".", 1)[0] + ".txt")

    image = cv2.imread(image_path)
    h, w = image.shape[:2]

    if not os.path.exists(label_path):
        continue

    boxes, classes = load_labels(label_path)

    for i in range(AUGMENTATIONS_PER_IMAGE):
        try:
            transformed = transform(image=image, bboxes=boxes, class_labels=classes)
            aug_image = transformed['image']
            aug_boxes = transformed['bboxes']
            aug_classes = transformed['class_labels']

            if not aug_boxes:
                continue  # Skip if augmentation removes all boxes

            output_img_name = f"{filename.rsplit('.', 1)[0]}_aug{i}.jpg"
            output_lbl_name = f"{filename.rsplit('.', 1)[0]}_aug{i}.txt"

            cv2.imwrite(os.path.join(OUTPUT_IMAGE_DIR, output_img_name), aug_image)
            save_labels(os.path.join(OUTPUT_LABEL_DIR, output_lbl_name), aug_boxes, aug_classes)

        except Exception as e:
            print(f"Error augmenting {filename}: {e}")

print("✅ Offline augmentation complete.")
