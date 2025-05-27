import json
import logging
import argparse
import rclpy
import time
from time import sleep
from rclpy.node import Node
from paho.mqtt import client as mqtt
from std_msgs.msg import String
import threading
import math

import numpy as np
import cv2
from ultralytics import YOLO 

# Global variables
myDroneId = "drone01"
publisher = ''
lock = threading.Lock()
status_subscriber = ''
currentStatus = ''

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
LATERAL_LIMIT = 30

CAMERA_HORIZONTAL_FOV = 64
CAMERA_VERTICAL_FOV = 41

DESIRED_DIST = 5
MAX_DIST = 1

X_OFFSET_THRESHOLD = 20  # pixels
Y_OFFSET_THRESHOLD = 30  # pixels
PIXEL_TO_METERS = 0.01   # scale factor to convert pixel error to movement

denmMessage = {
    "management": {
        "actionID": {
            "originatingStationID": 1798587532,
            "sequenceNumber": 0
        },
        "detectionTime": 1626453837.658,
        "referenceTime": 1626453837.658,
        "eventPosition": {
            "latitude": 40.637799251415686,
            "longitude": -8.652353364491056,
            "positionConfidenceEllipse": {
                "semiMajorConfidence": 0,
                "semiMinorConfidence": 0,
                "semiMajorOrientation": 0
            },
            "altitude": {
                "altitudeValue": 0,
                "altitudeConfidence": 1
            }
        },
        "validityDuration": 0,
        "stationType": 0
    },
    
    "situation": {
        "informationQuality": 7,
        "eventType": {
            "causeCode": 13
        }
    },
    "location": {
      "detectionZonesToEventPosition":{
        "traces": []
      },
      "eventPositionHeading":{
        "value": 360,
        "confidence":0 
      }

   }
}


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


yawRateToTurn = 0
distanceToPoint = 10000000

# class yoloTracking(Node):
#     def __init__(self, model_path="/sensors/yolo_model/best.pt", video_path="/sensors/yolo_model/Fullv2.mp4"):
#         self.model = YOLO(model_path)
#         self.cap = cv2.VideoCapture(video_path)

#         self.yaw_rate_buffer = []

#     def process_frame(self, frame):
#         # frame = cv2.resize(frame, (CAMERA_WIDTH, CAMERA_HEIGHT))
#         results = self.model(frame, conf=0.6, verbose=False)
#         boxes = results[0].boxes.xyxy.cpu().numpy()
#         classes = results[0].boxes.cls.cpu().numpy()
#         labels = [self.model.names[int(c)] for c in classes]

#         h, w = frame.shape[:2]
#         cx, cy = w // 2, h // 2

#         angles, centers = [], []

#         for idx, box in enumerate(boxes):
#             if labels[idx] == "Occupied":
#                 continue
#             x1, y1, x2, y2 = box
#             bx, by = (x1 + x2) // 2, (y1 + y2) // 2
#             dx, dy = bx - cx, by - cy
#             angle = 0 if dx == 0 else (90 - np.abs(np.degrees(np.arctan(dy / dx))))
#             if dx < 0: angle = -angle
#             angles.append(angle)
#             centers.append((bx, by))

#         return angles, centers, frame

#     def run(self):
#         global yawRateToTurn, distanceToPoint
#         while True:
#             ret, frame = self.cap.read()
#             if not ret:
#                 continue

#             gpu_frame = cv2.cuda_GpuMat()
#             gpu_frame.upload(frame)

#             # Resize on GPU (faster than CPU)
#             gpu_resized = cv2.cuda.resize(gpu_frame, (640, 480))

#             # Download back to CPU for display
#             frame = gpu_resized.download()

                
#             angles, centers, frame = self.process_frame(frame)

#             if angles:
#                 best_idx = min(range(len(angles)), key=lambda i: (abs(angles[i]), -angles[i]))
#                 best_angle = angles[best_idx]
#                 best_center = centers[best_idx]

#                 self.yaw_rate_buffer.append(best_angle)
#                 if len(self.yaw_rate_buffer) > 10:
#                     self.yaw_rate_buffer.pop(0)

#                 yawRateToTurn = sum(self.yaw_rate_buffer) / len(self.yaw_rate_buffer)
#                 distanceToPoint = math.hypot(best_center[0] - CAMERA_WIDTH // 2, best_center[1] - CAMERA_HEIGHT // 2)
#             else:
#                 yawRateToTurn = 0
#                 distanceToPoint = float('inf')

#             cv2.imshow("YOLO Tracking", frame)
#             if cv2.waitKey(1) & 0xFF == ord('q'):
#                 break
    # def __init__(self):
    #     self.model = YOLO("/sensors/yolo_model/best.pt")
    #     # self.cap = cv2.VideoCapture(0)
    #     self.cap = cv2.VideoCapture("/sensors/yolo_model/Fullv2.mp4")
    #     self.yawRateBuffer = []
    # def run(self): 
    #     global yawRateToTurn
    #     global distanceToPoint
    #     while True:
    #         ret, frame = self.cap.read()
    #         if not ret:
    #             #self.cap = cv2.VideoCapture("/sensors/yolo_model/Fullv2.mp4")
    #             continue
    #         # frame = cv2.imread("/sensors/yolo_model/yolo.PNG")
    #         frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_LINEAR)
            

    #         # Run YOLOv8 on the frame
    #         results = self.model(frame, conf=0.6, verbose=False)
    #         names = results[0].boxes.cls.cpu().numpy()  # Get class names
    #         names = [self.model.names[int(name)] for name in names]  # Convert class indices to names
    #         annotations = results[0].boxes.xyxy.cpu().numpy()  # Get bounding box coordinates
    #         #print(names)
    #         # Convert to a list of dictionaries
    #         # annotations_list = []
    #         # for box in annotations:
    #         #     x1, y1, x2, y2 = box
    #         #     annotations_list.append({
    #         #         "x1": int(x1),
    #         #         "y1": int(y1),
    #         #         "x2": int(x2),
    #         #         "y2": int(y2)
    #         #     })
    #         # Draw a line from the center of the image to the center of each bounding box
    #         h, w, _ = frame.shape
    #         #h,w = 480, 640
    #         center_x, center_y = w // 2, h // 2
    #         nameIndex = 0
    #         angles = []
    #         boxes = []
            
    #         for box in annotations:
    #             x1, y1, x2, y2 = box
    #             # x1 = box["x1"]
    #             # y1 = box["y1"]
    #             # x2 = box["x2"]
    #             # y2 = box["y2"]
                
    #             name = names[nameIndex]
    #             if(name == "Occupied"):
    #                 nameIndex += 1
    #                 continue
    #             box_center_x = (x1 + x2) // 2
    #             box_center_y = (y1 + y2) // 2
    #             # print(f"Box center: ({box_center_x}, {box_center_y})")
    #             # print(f"Image center: ({center_x}, {center_y})")
    #             # Draw a line from the center of the image to the center of the bounding box
                
    #             # Calculate the angle
    #             deltaX = box_center_x - center_x
    #             deltaY = box_center_y - center_y
    #             if(deltaX == 0):
    #                 angles.append(0)
    #                 boxes.append((box_center_x, box_center_y))
    #                 break
    #             deltaTan = deltaY / deltaX
    #             angle = 90 - np.abs(math.degrees(math.atan(deltaTan)))
    #             if(deltaX < 0):
    #                 angle = -angle
    #             angles.append(angle)
    #             boxes.append((box_center_x, box_center_y))
    #             nameIndex += 1
    #         closest_to_zero = lambda lst: min(lst, key=lambda x: (abs(x), -x))
    #         # Labda function to find the index of the closest value to zero in a set
    #         closest_to_zero_index = lambda lst: min(range(len(lst)), key=lambda i: (abs(lst[i]), -lst[i]))
    #         if(len(angles) > 0):
    #             self.yawRateBuffer.append(closest_to_zero(angles))
    #             if len(self.yawRateBuffer) > 10:
    #                 self.yawRateBuffer.pop(0)
    #             yawRateToTurn = sum(self.yawRateBuffer) / len(self.yawRateBuffer)
    #             print(boxes[closest_to_zero_index(angles)])
    #             distanceToPoint =  math.sqrt((boxes[closest_to_zero_index(angles)][0] - center_x) ** 2 + (boxes[closest_to_zero_index(angles)][1] - center_y) ** 2)
    #         else:
    #             yawRateToTurn = 0
    #             distanceToPoint = 10000000
    #         cv2.imshow("YOLO Tracking", frame)
    #         if cv2.waitKey(1) & 0xFF == ord('q'):
    #             break
            
           

class StatusSubscriber(Node):    
    def __init__(self):
        global myDroneId
        name = 'status_subscriber_' + myDroneId

        super().__init__(name)
        self.subscription = self.create_subscription(
            String,
            '/status',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg): 
        global currentStatus
                         
        currentStatus = msg.data
        
class TelemSubscriber(Node):
    def __init__(self):
        global myDroneId
        name = 'telem_subscriber_' + myDroneId

        super().__init__(name)
        self.subscription = self.create_subscription(
            String,
            '/telem',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg): 
        global currentTelem
                         
        currentTelem = msg.data
        
# class MQTTClient:
#     def __init__(self, broker="127.0.0.1", port=1883, topic="sensors/yolo_tracking"):
#         self.broker = broker
#         self.port = port
#         self.topic = topic
#         self.data = None
#         self.client = self.connect_mqtt()
#         self.subscribe()
#         self.client.loop()

#     def connect_mqtt(self) -> mqtt_client:
#         def on_connect(client, userdata, flags, rc):
#             if rc == 0:
#                 logger.info("Connected to MQTT Broker!")
#             else:
#                 logger.error("Failed to connect, return code %d\n", rc)

#         client = mqtt_client.Client()
#         # client.username_pw_set(username, password)
#         client.on_connect = on_connect
#         client.connect(self.broker, self.port)
#         return client


#     def subscribe(self) -> None:
#         def on_message(client, userdata, message):
#             print(f"Received message '{message.payload.decode()}' on topic '{message.topic}'")
#             self.data = message.payload.decode()

#         self.client.subscribe(self.topic)
#         self.client.on_message = on_message

class MQTTClientHandler:
    def __init__(self, broker_ip, broker_port, topic, client_id=None, on_message_callback=None):
        self.broker_ip = broker_ip
        self.broker_port = broker_port
        self.topic = topic
        self.client_id = client_id or f"client_{topic}"
        self.on_message_callback = on_message_callback

        self._client = mqtt.Client(client_id=self.client_id)
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message

        self._data_lock = threading.Lock()
        self._latest_data = {}

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print(f"[{self.client_id}] Connected to {self.broker_ip}:{self.broker_port}")
            client.subscribe(self.topic)
        else:
            print(f"[{self.client_id}] Connection failed with code {rc}")

    def _on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            with self._data_lock:
                self._latest_data = payload
            if self.on_message_callback:
                self.on_message_callback(payload)
        except json.JSONDecodeError:
            print(f"[{self.client_id}] Received invalid JSON")

    def connect(self):
        self._client.connect(self.broker_ip, self.broker_port, 60)
        self._client.loop_start()

    def get_latest_data(self):
        with self._data_lock:
            return self._latest_data.copy()

    def publish(self, topic=None, payload=None):
        if payload is None or not isinstance(payload, dict):
            print(f"[{self.client_id}] Invalid publish payload")
            return
        topic = topic or self.topic
        self._client.publish(topic, json.dumps(payload))

denmMqtt = MQTTClientHandler(
    broker_ip="192.168.122.166",
    broker_port=1883,
    topic="vanetza/in/denm",
    client_id="drone01_client",
    on_message_callback=lambda data: print(f"Received data: {data}")
)
denmMqtt.connect()
logger.info("Logged sucessfully")

def yolo_msg_handle(data):
    global yawRateToTurn
    global distanceToPoint
    # print(f"Received YOLO message: {data}")
    try:
        # data = json.loads(data)
        if 'yawRate' in data and 'distance' in data:
            yawRateToTurn = data['yawRate']
            distanceToPoint = data['distance']
            print(f"Yaw Rate: {yawRateToTurn}, Distance: {distanceToPoint}")
        else:
            print("Invalid YOLO message format")
    except json.JSONDecodeError:
        print("Failed to decode YOLO message JSON")
    
    

yoloMqtt = MQTTClientHandler(
    broker_ip="192.168.122.166",
    broker_port=1883,
    topic="sensor/yolo_tracking",
    client_id="drone01_yolo_client",
    on_message_callback=yolo_msg_handle
    )

yoloMqtt.connect()

def current_time_millis():
    return int(round(time.time() * 1000))

def base_cmd(droneId = None):
    global myDroneId
    if droneId == None:
        droneId = myDroneId
    return {'droneId': droneId, 'mode': 'action', 'timestamp': current_time_millis()}

def publish_cmd(cmd):
    global publisher
    msg = String()
    msg.data = json.dumps(cmd)
    publisher.publish(msg)

def arm(droneId=None):
    global myDroneId
    if droneId == None:
        droneId = myDroneId
    cmd = base_cmd(droneId = droneId)
    cmd['cmd'] = 'arm'
    publish_cmd(cmd)
    
def takeoff_cmd(droneId=None,alt=3):
    global myDroneId
    if droneId == None:
        droneId = myDroneId
    cmd = base_cmd(droneId = droneId)
    cmd['cmd'] = 'takeoff'
    cmd['alt'] = alt
    publish_cmd(cmd)
    print("Taking off drone to " + str(alt) + " meters")
    
def turn(degree, droneId):
    global myDroneId
    if droneId == None:
        droneId = myDroneId
    cmd = base_cmd(droneId = droneId)
    cmd["mode"] = "custom"
    cmd["cmd"] = "turn"
    cmd["deg"] = degree
    if degree > 0:
        print("Turning right " + str(degree) + " degrees")
    else:
        print("Turning left " + str(degree) + " degrees")
    publish_cmd(cmd)
    
def move(x, y, z, droneId, yaw=None,speed=1):
    global myDroneId
    if droneId == None:
        droneId = myDroneId
    cmd = base_cmd(droneId = droneId)
    cmd["mode"] = "custom"
    cmd["cmd"] = "move"
    cmd["x"] = x
    cmd["y"] = y
    cmd["z"] = z    
    cmd["yaw"] = yaw
    cmd["speed"] = speed
    print(f"Moving forward {y} meters")
    publish_cmd(cmd)
    
def goto_cmd(lat, lon, alt=None, yaw=None, speed=None,droneId=None):
    global myDroneId
    if droneId == None:
        droneId = myDroneId
    cmd = base_cmd(droneId=droneId)
    cmd["mode"] = "action"
    cmd["cmd"] = "goto"
    cmd["lat"] = lat
    cmd["lon"] = lon
    if alt != None:
        cmd["alt"] = alt  
    cmd["yaw"] = yaw
    if speed != None:
        cmd['speed'] = speed
    publish_cmd(cmd)
    
def cancel_cmd(droneId=None):
    global myDroneId
    if droneId == None:
        droneId = myDroneId
    cmd = base_cmd(droneId=droneId)
    cmd['mode'] = 'custom'
    cmd['cmd'] = 'cancel'
    print("Cancelling previous command")
    publish_cmd(cmd)

def center_drone(object_x_center):
    pixel_offset = object_x_center - (CAMERA_WIDTH / 2)
    angle_offset = (pixel_offset / CAMERA_WIDTH) * CAMERA_HORIZONTAL_FOV
    return angle_offset

def new_coords_latlon(lat, lon, heading_deg, distance_m):
    # Earth radius (approx)
    R = 6371000  
    heading_rad = math.radians(heading_deg)
    lat_rad = math.radians(lat)
    
    # Negative distance for going backwards
    d_lat = -distance_m * math.cos(heading_rad) / R
    d_lon = -distance_m * math.sin(heading_rad) / (R * math.cos(lat_rad))
    
    new_lat = lat + math.degrees(d_lat)
    new_lon = lon + math.degrees(d_lon)
    return new_lat, new_lon
     
def waitForTask(cmd=None,state=None,droneId=None):    
    global currentStatus
    global myDroneId
    global status_subscriber
    global lock
    if droneId == None:
        droneId = myDroneId
    while True:        
        lock.acquire()   
        rclpy.spin_once(status_subscriber, timeout_sec=0.1)
        # print("cmd :",cmd)

        # print(currentStatus)
        try:
            tmp = json.loads(currentStatus)
        except:
            tmp = {}
        lock.release()
        if 'command' in tmp:      
            if tmp['droneId'] == droneId and tmp['command'] == cmd and tmp['state'] == state :           
                break

def get_pixel_offsets(center, width=CAMERA_WIDTH, height=CAMERA_HEIGHT):
    image_center = (width / 2, height / 2)
    offset_x = center[0] - image_center[0]
    offset_y = center[1] - image_center[1]
    return offset_x, offset_y


def main(args=None):
    global myDroneId
    global publisher
    global status_subscriber
    global telem_subscriber
    global yawRateToTurn
    global distanceToPoint
    
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--drone', required=False)
    parser.add_argument('-i', '--id', required=False)
    
        
    cli_args = parser.parse_args()
    
    if cli_args.drone:
        myDroneId = f"drone0{cli_args.drone}"
    if cli_args.id:
        objectId = cli_args.id
    else:
        objectId = ""
    
    # TODO create a thread to run the YOLO detection on loop
    
    # yolotrack = yoloTracking()
    
    # yolothr = threading.Thread(target=yolotrack.run)
    # yolothr.start()
    
    
    
    rclpy.init(args=args)

    # Create a ROS node
    node = rclpy.create_node('publisher' + myDroneId)
    
    print("Drone ID: " + myDroneId)
    # Create a publisher for commands, a status subscriber and a telemetry subscriber
    publisher = node.create_publisher(String, '/cmd', 10)
    status_subscriber = StatusSubscriber()
    telem_subscriber = TelemSubscriber()
    
    tries = 0
    
    cancel_cmd()
    
    
    
    # Wait for the drone to be ready
    while True:
        lock.acquire()
        print("Waiting for drone to be ready")
        
        rclpy.spin_once(telem_subscriber, timeout_sec=0.5)
        telem = {}
        while telem == {}:
            try:
                print(currentTelem)
                telem = json.loads(currentTelem)
            except:
                telem = {}
            time.sleep(0.1)
        lock.release()
        if telem['droneId'] == myDroneId:
            # Arm and takeoff drone in case it's landed
            if not telem['armed']:
                # Take off
                arm(myDroneId)
                print("Waiting for drone to be armed")
                waitForTask('arm','success', myDroneId)
                print("Arming drone")
            print(" TELEM : ", telem["position"])
            while(telem['position']["height"] == None):
                time.sleep(0.1)
            if telem['height'] == 0 or int(telem["position"]["height"]) == 0:
                takeoff_cmd(myDroneId)
                waitForTask('takeoff','finish', myDroneId)
            break
        
    # thr = threading.Thread(target=client.client.loop_forever)
    # thr.start()
    # 40.633950, -8.659607
    goto_cmd(40.633950,-8.659607,yaw=140,speed=2.5, droneId=myDroneId)
    waitForTask('goto', 'finish', myDroneId)
    turn(39.0, myDroneId)  # Turn to face the object
    waitForTask('turn', 'finish', myDroneId)
    move(0, 4, 0, myDroneId, speed=0.1)  # Move forward
    denmMessage["management"]["eventPosition"]["latitude"] = telem["position"]["lat"]
    denmMessage["management"]["eventPosition"]["longitude"] = telem["position"]["lon"]

    denmMqtt.publish(denmMqtt.topic, denmMessage)
    return
    
    # First task
    
    # goto_cmd(40.63397,-8.659615, droneId=myDroneId)
    # waitForTask('goto', 'finish', myDroneId)
    
    # while True:

        
    #     while True:
    #         time.sleep(0.1)
    #         print("\r Current YAW: {} ; Distance: {}".format(yawRateToTurn, distanceToPoint))
    #         if distanceToPoint > 50:
    #             # If the distance is greater than 10 pixels, we need to turn
    #             turn(yawRateToTurn, myDroneId)
    #             waitForTask('turn', 'finish', myDroneId)
    #             print("\r", yawRateToTurn, distanceToPoint)
    #             move(0,100,0, myDroneId,speed=0.01)
    #             while(np.abs(0-yawRateToTurn) < 10 and distanceToPoint > 50):
    #                 # print(currentStatus)
    #                 time.sleep(1)
    #                 print("\r", yawRateToTurn, distanceToPoint)
    #                 continue
    #             print("PASSED ", yawRateToTurn, distanceToPoint)
    #             cancel_cmd()
    #             waitForTask("cancel", "success", myDroneId)
    #             if distanceToPoint < 50:
    #                 cancel_cmd(myDroneId)
    #                 break
    #     # TODO MQTT STUFF
    #     print(telem)
    #     denmMessage["management"]["eventPosition"]["latitude"] = telem["position"]["lat"]
    #     denmMessage["management"]["eventPosition"]["longitude"] = telem["position"]["lon"]

    #     denmMqtt.publish(denmMqtt.topic, denmMessage)
    #     return
                
                
if __name__ == '__main__':
    main()
