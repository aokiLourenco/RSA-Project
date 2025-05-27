import cv2
from ultralytics import YOLO
import math
import numpy as np
# Load YOLOv8 model (you can use your custom model path here)
model = YOLO("/home/to/UA/4-Ano/2-Semestre/RSA/Project/RSA-Project/YOLOStuff/models/v2/best.pt", verbose=True)

# Open the default webcam (0). Change to 1 or another index for external cameras
cap = cv2.VideoCapture("/home/to/UA/4-Ano/2-Semestre/RSA/Project/RSA-Project/YOLOStuff/Try3.mp4")  # Use a video file for testing, change to 0 for webcam


# Yaw Rate 
degrees_for_pixel = 35/320


def movement_to_degrees(rotation):
    print(degrees_for_pixel)
    return rotation*degrees_for_pixel



if not cap.isOpened():
    print("Error: Cannot open webcam")
    exit()
yawRateBuffer = []
while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Run YOLOv8 on the frame
    results = model(frame, conf=0.6, verbose=False)

    # Annotate the frame with detection boxes
    annotated_frame = results[0].plot()

    # Consider the center of the image as the origin (0, 0)
    # Calculate the diference between the center and all the bonding boxes

    # Get the bounding boxes coordinates and names
    # Save into a json file
    names = results[0].boxes.cls.cpu().numpy()  # Get class names
    names = [model.names[int(name)] for name in names]  # Convert class indices to names
    annotations = results[0].boxes.xyxy.cpu().numpy()  # Get bounding box coordinates
    # Convert to a list of dictionaries
    # Draw a line from the center of the image to the center of each bounding box
    h, w, _ = frame.shape
    center_x, center_y = w // 2, h // 2
    nameIndex = 0
    angles = []
    boxes = []
    
    for box in annotations:
        x1, y1, x2, y2 = box
        
        name = names[nameIndex]
        if(name == "Occupied"):
            nameIndex += 1
            continue
        box_center_x = (x1 + x2) // 2
        box_center_y = (y1 + y2) // 2
        # print(f"Box center: ({box_center_x}, {box_center_y})")
        # print(f"Image center: ({center_x}, {center_y})")
        # Draw a line from the center of the image to the center of the bounding box
        
        # Calculate the angle
        deltaX = box_center_x - center_x
        deltaY = box_center_y - center_y
        if(deltaX == 0):
            deltaX = 1
        deltaTan = deltaY / deltaX
        angle = math.degrees(math.atan(deltaTan))
        angle = 90 - np.abs(angle)
        if(deltaX < 0):
            angle = -angle
        angles.append(angle)
        boxes.append((box_center_x, box_center_y))
        nameIndex += 1
        
        
        
        cv2.line(annotated_frame, (center_x, center_y), (int(box_center_x), int(box_center_y)), (0, 255, 0), 2) # TODO delete later

    closest_to_zero = lambda lst: min(lst, key=lambda x: (abs(x), -x))
    # Labda function to find the index of the closest value to zero in a set
    closest_to_zero_index = lambda lst: min(range(len(lst)), key=lambda i: (abs(lst[i]), -lst[i]))
    if(len(angles) > 0):
        yawRateBuffer.append(closest_to_zero(angles))
        if len(yawRateBuffer) > 10:
            yawRateBuffer.pop(0)
        # Calculate the average yaw rate
        yawRateToTurn = sum(yawRateBuffer) / len(yawRateBuffer)
        print(boxes[closest_to_zero_index(angles)])
        distanceToPoint =  math.sqrt((boxes[closest_to_zero_index(angles)][0] - center_x) ** 2 + (boxes[closest_to_zero_index(angles)][1] - center_y) ** 2)
    else:
        yawRateToTurn = 0
        distanceToPoint = 10000000
    
    # Draw the yaw rate and distance on the frame
    cv2.putText(annotated_frame, f"Yaw Rate: {yawRateToTurn:.2f} degrees", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(annotated_frame, f"Distance: {distanceToPoint:.2f} pixels", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    # Draw a 10 pixel radius circle around the center of the image
    cv2.circle(annotated_frame, (center_x, center_y), 50, (0, 255, 0), -1)
    
    if distanceToPoint < 50:
        cv2.putText(annotated_frame, "Object is close!", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        while True:
            # Wait for 1 second
            cv2.waitKey(1000)
            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    # Display the result
    annotated_frame = cv2.resize(annotated_frame, (1024, 720), interpolation=cv2.INTER_LINEAR)
    cv2.imshow("YOLOv8 Real-Time Detection", annotated_frame)

    # Break on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()
