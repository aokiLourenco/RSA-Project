import os
from ultralytics import YOLO

# Configuration
MODEL_ARCH = "yolov8n.pt"  # You can switch to yolov8s.pt or yolov8m.pt for more accuracy
DATA_YAML = "dataset/data.yaml"
EPOCHS = 30
BATCH_SIZE = 16
IMAGE_SIZE = 640
PROJECT_DIR = "training_runs"
EXPERIMENT_NAME = "vacant_and_occupied_detector_v2"

def main():
    print("Loading model...")
    model = YOLO(MODEL_ARCH)

    print("Starting training...")
    model.train(
        cache=False,
        data=DATA_YAML,
        epochs=EPOCHS,
        batch=BATCH_SIZE,
        imgsz=IMAGE_SIZE,
        project=PROJECT_DIR,
        name=EXPERIMENT_NAME,
        exist_ok=True  # Overwrite existing run if it exists
    )

    print(f"✅ Training complete. Results saved in {os.path.join(PROJECT_DIR, EXPERIMENT_NAME)}")

if __name__ == "__main__":
    main()
