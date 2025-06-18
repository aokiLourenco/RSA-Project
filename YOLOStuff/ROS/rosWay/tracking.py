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
        "actionId": {
            "originatingStationId": 1,
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
            "ccAndScc": {   
                "reserved13": 13
            }
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
    broker_ip="192.168.159.29",
    broker_port=1883,
    # topic="vanetza/in/denm",
    topic="autoware/goal_remap",
    client_id="drone01",
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
    
    
    
    rclpy.init(args=args)

    # Create a ROS node
    node = rclpy.create_node('publisher' + myDroneId)
    print("\033[2J\033[H\r")
    print("Drone ID: " + myDroneId)
    # Create a publisher for commands, a status subscriber and a telemetry subscriber
    publisher = node.create_publisher(String, '/cmd', 10)
    status_subscriber = StatusSubscriber()
    telem_subscriber = TelemSubscriber()
    
    cancel_cmd()
    
    
    
    # Wait for the drone to be ready
    while True:
        lock.acquire()
        print("Waiting for drone to be ready")
        
        rclpy.spin_once(telem_subscriber, timeout_sec=0.5)
        telem = {}
        while telem == {}:
            try:
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
            print(" TELEM : ", telem["position"])
            while(telem['position']["height"] == None):
                time.sleep(0.1)
            if telem['height'] == 0 or int(telem["position"]["height"]) == 0:
                print("Taking off drone")
                takeoff_cmd(myDroneId)
                waitForTask('takeoff','finish', myDroneId)
            break
    print("Moving to Parking Lot")    
    #First task
    
    goto_cmd(40.63397,-8.659615, droneId=myDroneId)
    waitForTask('goto', 'finish', myDroneId)
    
    while True:

        
        while True:
            time.sleep(0.1)
            print("\r Current YAW: {} ; Distance: {}".format(yawRateToTurn, distanceToPoint))
            if distanceToPoint > 50:
                # If the distance is greater than 10 pixels, we need to turn
                turn(yawRateToTurn, myDroneId)
                waitForTask('turn', 'finish', myDroneId)
                print("\r", yawRateToTurn, distanceToPoint)
                move(0,100,0, myDroneId,speed=0.01)
                while(np.abs(0-yawRateToTurn) < 10 and distanceToPoint > 50):
                    # print(currentStatus)
                    time.sleep(1)
                    print("\r", yawRateToTurn, distanceToPoint)
                    continue
                print("PASSED ", yawRateToTurn, distanceToPoint)
                cancel_cmd()
                waitForTask("cancel", "success", myDroneId)
                if distanceToPoint < 50:
                    cancel_cmd(myDroneId)
                    break
        # TODO MQTT STUFF
        print(telem)
        denmMessage["management"]["eventPosition"]["latitude"] = telem["position"]["lat"]
        denmMessage["management"]["eventPosition"]["longitude"] = telem["position"]["lon"]

        denmMqtt.publish(denmMqtt.topic, denmMessage)
        return
                
                
if __name__ == '__main__':
    main()
