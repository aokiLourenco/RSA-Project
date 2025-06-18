# import cv2
# from ultralytics import YOLO
# import math
# import numpy as np
# import paho.mqtt.client as mqtt
# import json
# # Load YOLOv8 model (you can use your custom model path here)
# model = YOLO("models/best.pt")  

# # Open the default webcam (0). Change to 1 or another index for external cameras
# cap = cv2.VideoCapture("imagesTest/Fullv2.mp4")  # Use a video file for testing, change to 0 for webcam


# MQTT_BROKER = "192.168.122.166"  # Change to your broker's IP or hostname
# MQTT_PORT = 1883
# MQTT_TOPIC = "sensor/yolo_tracking"

# client = mqtt.Client()
# client.connect(MQTT_BROKER, MQTT_PORT, 60)
# client.loop_start()


# if not cap.isOpened():
#     print("Error: Cannot open webcam")
#     exit()


# def publish_data(yaw_rate, distance):
#     payload = {
#         "yawRate": yaw_rate,
#         "distance": distance
#     }
#     client.publish(MQTT_TOPIC, json.dumps(payload))

# yawRateBuffer = []
# while True:
#     ret, frame = cap.read()
#     if not ret:
#         break
    
#     frame = cv2.resize(frame, (640, 480))  # Resize frame to 640x480 for consistency
#     # Run YOLOv8 on the frame
#     results = model(frame, conf=0.6, verbose=False)
#     annotated_frame = results[0].plot()  # Get the annotated frame
#     names = results[0].boxes.cls.cpu().numpy()  # Get class names
#     names = [model.names[int(name)] for name in names]  # Convert class indices to names
#     annotations = results[0].boxes.xyxy.cpu().numpy()  # Get bounding box coordinates
#     #print(names)
#     # Convert to a list of dictionaries
#     # Draw a line from the center of the image to the center of each bounding box
#     h, w, _ = frame.shape
#     center_x, center_y = w // 2, h // 2
#     nameIndex = 0
#     angles = []
#     boxes = []
#     for box in annotations:
#         x1, y1, x2, y2 = box
        
#         name = names[nameIndex]
#         if(name == "Occupied"):
#             nameIndex += 1
#             continue
#         box_center_x = (x1 + x2) // 2
#         box_center_y = (y1 + y2) // 2
#         # print(f"Box center: ({box_center_x}, {box_center_y})")
#         # print(f"Image center: ({center_x}, {center_y})")
#         # Draw a line from the center of the image to the center of the bounding box
        
#         # Calculate the angle
#         deltaX = box_center_x - center_x
#         deltaY = box_center_y - center_y
#         if(deltaX == 0):
#             deltaX = 1
#         deltaTan = deltaY / deltaX
#         angle = math.degrees(math.atan(deltaTan))
#         angle = 90 - np.abs(angle)
#         if(deltaX < 0):
#             angle = -angle
#         angles.append(angle)
#         boxes.append((box_center_x, box_center_y))
#         nameIndex += 1
        
        
        
#         cv2.line(annotated_frame, (center_x, center_y), (int(box_center_x), int(box_center_y)), (0, 255, 0), 2) # TODO delete later
#     closest_to_zero = lambda lst: min(lst, key=lambda x: (abs(x), -x))
#     # Labda function to find the index of the closest value to zero in a set
#     closest_to_zero_index = lambda lst: min(range(len(lst)), key=lambda i: (abs(lst[i]), -lst[i]))
#     if(len(angles) > 0):
#         yawRateBuffer.append(closest_to_zero(angles))
#         if len(yawRateBuffer) > 10:
#             yawRateBuffer.pop(0)
#         # Calculate the average yaw rate
#         yawRateToTurn = sum(yawRateBuffer) / len(yawRateBuffer)
#         print(boxes[closest_to_zero_index(angles)])
#         distanceToPoint =  math.sqrt((boxes[closest_to_zero_index(angles)][0] - center_x) ** 2 + (boxes[closest_to_zero_index(angles)][1] - center_y) ** 2)
#     else:
#         yawRateToTurn = 0
#         distanceToPoint = 10000000
#     publish_data(yawRateToTurn, distanceToPoint)
#     # Draw the yaw rate and distance on the frame
#     cv2.putText(annotated_frame, f"Yaw Rate: {yawRateToTurn:.2f} degrees", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
#     cv2.putText(annotated_frame, f"Distance: {distanceToPoint:.2f} pixels", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
#     # Draw a 10 pixel radius circle around the center of the image
#     cv2.circle(annotated_frame, (center_x, center_y), 30, (0, 255, 0), -1)
    
#     # Display the annotated frame
#     cv2.imshow("YOLOv8 Detection", annotated_frame)
    
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# # Release the camera and close all windows
# cap.release()
# cv2.destroyAllWindows()


import cv2
from ultralytics import YOLO
import math
import numpy as np
import paho.mqtt.client as mqtt
import json

model = YOLO("models/best.pt")
# cap = cv2.VideoCapture("imagesTest/Fullv2.mp4")
cap = cv2.VideoCapture(0)  # Use webcam, change to video file path if needed

MQTT_BROKER = "127.0.0.1"
MQTT_PORT = 1883
MQTT_TOPIC = "sensor/yolo_tracking"

client = mqtt.Client()
client.connect(MQTT_BROKER, MQTT_PORT, 60)
client.loop_start()

if not cap.isOpened():
    print("Error: Cannot open video stream")
    exit()

def publish_data(yaw_rate, distance):
    payload = {
        "yawRate": yaw_rate,
        "distance": distance
    }
    client.publish(MQTT_TOPIC, json.dumps(payload))

yawRateBuffer = []

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.resize(frame, (640, 480))
    results = model(frame, conf=0.6, verbose=False)
    annotated_frame = results[0].plot()

    names = [model.names[int(i)] for i in results[0].boxes.cls.cpu().numpy()]
    boxes_xyxy = results[0].boxes.xyxy.cpu().numpy()

    h, w, _ = frame.shape
    center_x, center_y = w // 2, h // 2

    angles = []
    distances = []
    centers = []
    nameIndex = 0

    for box in boxes_xyxy:
        x1, y1, x2, y2 = box
        name = names[nameIndex]
        nameIndex += 1
        if name == "Occupied":
            continue

        box_cx = (x1 + x2) / 2
        box_cy = (y1 + y2) / 2

        dx = box_cx - center_x
        dy = box_cy - center_y
        dx = 1 if dx == 0 else dx  # avoid division by zero
        angle = 90 - abs(math.degrees(math.atan(dy / dx)))
        angle = -angle if dx < 0 else angle

        distance = math.sqrt(dx**2 + dy**2)

        angles.append(angle)
        distances.append(distance)
        centers.append((box_cx, box_cy))

        # Visualization
        cv2.line(annotated_frame, (center_x, center_y), (int(box_cx), int(box_cy)), (0, 255, 0), 2)

    if angles:
        # Scoring based on composite metric
        weight_angle = 1.0
        weight_distance = 0.3  # Adjust based on desired influence

        scores = [
            weight_angle * abs(angles[i]) + weight_distance * distances[i]
            for i in range(len(angles))
        ]

        best_idx = scores.index(min(scores))
        best_angle = angles[best_idx]
        best_distance = distances[best_idx]

        yawRateBuffer.append(best_angle)
        if len(yawRateBuffer) > 10:
            yawRateBuffer.pop(0)

        yawRateToTurn = sum(yawRateBuffer) / len(yawRateBuffer)
        distanceToPoint = best_distance

        print(f"Selected Box Center: {centers[best_idx]}")
    else:
        yawRateToTurn = 0
        distanceToPoint = 1e7

    publish_data(yawRateToTurn, distanceToPoint)

    # Display info on screen
    cv2.putText(annotated_frame, f"Yaw Rate: {yawRateToTurn:.2f} deg", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(annotated_frame, f"Distance: {distanceToPoint:.2f} px", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.circle(annotated_frame, (center_x, center_y), 30, (0, 255, 0), -1)

    cv2.imshow("YOLOv8 Detection", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
