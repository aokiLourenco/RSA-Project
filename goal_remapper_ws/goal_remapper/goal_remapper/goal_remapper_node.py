#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import json
import paho.mqtt.client as mqtt
import math
from geometry_msgs.msg import Quaternion
import subprocess
import threading
import select
import sys
import signal
import os

class GoalRemapper(Node):
    def __init__(self):
        super().__init__('goal_remapper')
        
        # Publish initial pose ONCE before connecting to MQTT
        self.publish_initial_pose()

        # Declare parameters
        self.declare_parameter('mqtt_broker_address', 'localhost')
        self.declare_parameter('mqtt_broker_port', 1883)
        self.declare_parameter('mqtt_topic', 'autoware/goal_remap')
        # Declare map origin parameters
        self.declare_parameter('map_origin.latitude', 40.634646)
        self.declare_parameter('map_origin.longitude', -8.659986)
        self.declare_parameter('map_origin.elevation', 0.0)

        # Get parameters
        mqtt_broker = self.get_parameter('mqtt_broker_address').value
        mqtt_port = self.get_parameter('mqtt_broker_port').value
        mqtt_topic = self.get_parameter('mqtt_topic').value

        # Get map origin
        self.map_origin_lat = self.get_parameter('map_origin.latitude').value
        self.map_origin_lon = self.get_parameter('map_origin.longitude').value
        self.map_origin_elev = self.get_parameter('map_origin.elevation').value

        # Create publishers
        self.original_goal_pub = self.create_publisher(
            PoseStamped, 
            '/planning/mission_planning/original_goal', 
            10)
        
        self.remapped_goal_pub = self.create_publisher(
            PoseStamped, 
            '/planning/mission_planning/goal', 
            10)
        
        # Create subscriber to intercept original goals
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/planning/mission_planning/goal', 
            self.goal_callback,
            10)
        
        # MQTT setup
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        
        # Remapping state
        self.remap_active = False
        self.remapped_goal = None
        self.goal_published = False  # New flag to track if goal was published
        self._shutdown_requested = False  # Flag to track shutdown
        
        # Connect to MQTT broker
        try:
            self.get_logger().info(f"Connecting to MQTT broker at {mqtt_broker}:{mqtt_port}")
            self.mqtt_client.connect(mqtt_broker, mqtt_port, 60)
            self.mqtt_client.subscribe(mqtt_topic)
            self.mqtt_client.loop_start()
            self.mqtt_connected = True
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT broker: {str(e)}")
            self.get_logger().warn("Will continue without MQTT functionality. Goal remapping will be disabled.")
            self.mqtt_connected = False
        
        self.get_logger().info('Goal Remapper node initialized')
    
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("Connected to MQTT broker")
        else:
            self.get_logger().error(f"Failed to connect to MQTT broker with code: {rc}")
    
    def on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode('utf-8')
            self.get_logger().info(f"Received MQTT with a DENM message: {payload}")
            msg_dict = json.loads(payload)

            # Detect DENM by structure (no 'command', but has 'management' and 'eventPosition')
            if (
                'command' not in msg_dict and
                'management' in msg_dict and
                'eventPosition' in msg_dict['management']
            ):
                self.process_denm_message(msg_dict)
            else:
                self.process_mqtt_message(msg_dict)



        except json.JSONDecodeError:
            self.get_logger().error(f"Failed to parse MQTT message as JSON")
        except Exception as e:
            self.get_logger().error(f"Error processing MQTT message: {str(e)}")

    def publish_initial_pose(self):
        """Publish initial pose command"""
        try:
            self.get_logger().info("Publishing initial pose to /initialpose3d (once)...")
            subprocess.run([
                "ros2", "topic", "pub", "--once", "/initialpose3d", "geometry_msgs/PoseWithCovarianceStamped",
                "{header: {stamp: {sec: 1748258635, nanosec: 780595328}, frame_id: 'map'},"
                "pose: {pose: {position: {x: 36.7392578125, y: -46.01408004760742, z: 2.450119733810425},"
                "orientation: {x: 0.0, y: 0.0, z: -0.3658047030231997, w: 0.9306916348856418}},"
                "covariance: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0,"
                "0.0, 1.0, 0.0, 0.0, 0.0, 0.0,"
                "0.0, 0.0, 0.01, 0.0, 0.0, 0.0,"
                "0.0, 0.0, 0.0, 0.01, 0.0, 0.0,"
                "0.0, 0.0, 0.0, 0.0, 0.01, 0.0,"
                "0.0, 0.0, 0.0, 0.0, 0.0, 0.2]}}"
            ], check=True, stderr=subprocess.DEVNULL)
        except Exception as e:
            self.get_logger().error(f"Failed to publish initial pose: {str(e)}")

    def process_denm_message(self, msg_dict):
        """Process incoming DENM MQTT messages and remap goal."""
        try:
            event_pos = msg_dict["management"]["eventPosition"]
            latitude = float(event_pos["latitude"])
            longitude = float(event_pos["longitude"])
            altitude = float(event_pos["altitude"]["altitudeValue"])

            # Default heading
            heading = 0.0
            if (
                "location" in msg_dict and
                "eventPositionHeading" in msg_dict["location"] and
                "value" in msg_dict["location"]["eventPositionHeading"]
            ):
                heading = float(msg_dict["location"]["eventPositionHeading"]["value"])

            # Convert lat/lon to x/y using map origin
            x, y = self.latlon_to_xy(latitude, longitude)

            # Create PoseStamped message
            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = 'map'

            goal_msg.pose.position.x = x
            goal_msg.pose.position.y = y
            goal_msg.pose.position.z = altitude

            # Convert heading to quaternion
            goal_msg.pose.orientation = self.heading_to_quaternion(heading)

            # Set remapping state and publish
            self.remapped_goal = goal_msg
            self.remap_active = True
            self.goal_published = False
            self.publish_goal()
            self.get_logger().info('Goal remapping activated from DENM message')

            # Start a timer to engage after 3 seconds (one-shot)
            self._engage_timer = self.create_timer(3.0, self._engage_autoware_oneshot)

        except Exception as e:
            self.get_logger().error(f"Failed to process DENM message: {str(e)}")

    def _engage_autoware_oneshot(self):
        self.engage_autoware()
        self._engage_timer.cancel()
        self.get_logger().info("Goal remapping cycle completed - waiting for next MQTT message")

    def engage_autoware(self):
        """Publish engage command to Autoware after delay."""
        try:
            self.get_logger().info("Publishing engage command to Autoware...")
            command = '''ros2 topic pub /autoware/engage autoware_auto_vehicle_msgs/msg/Engage '{engage: True}' -1'''
            subprocess.run(command, shell=True, stderr=subprocess.DEVNULL)
        except Exception as e:
            self.get_logger().error(f"Failed to publish engage command: {str(e)}")

    def latlon_to_xy(self, lat, lon):
        """
        Convert latitude and longitude to local Cartesian x, y using equirectangular approximation.
        Assumes small area around map origin.
        """
        # Constants
        R = 6378137.0  # Earth radius in meters

        lat0 = math.radians(self.map_origin_lat)
        lon0 = math.radians(self.map_origin_lon)
        lat = math.radians(lat)
        lon = math.radians(lon)

        dlat = lat - lat0
        dlon = lon - lon0

        x = dlon * math.cos(lat0) * R
        y = dlat * R
        return x, y

    def goal_callback(self, msg):
        """Handle incoming goal messages"""
        self.get_logger().debug('Received goal message')
        
        # Always republish to original topic for reference/debugging
        self.original_goal_pub.publish(msg)
        
        # Only republish the original goal, don't republish remapped goals here
        # This prevents continuous republishing of remapped goals
        if not self.remap_active:
            self.remapped_goal_pub.publish(msg)
            self.get_logger().debug('Passed through original goal')
    
    def process_mqtt_message(self, msg_dict):
        """Process incoming MQTT messages"""
        if 'command' not in msg_dict:
            self.get_logger().error('MQTT message missing command field')
            return
        
        command = msg_dict['command']
        
        if command == 'activate_remap':
            self.handle_activate_remap(msg_dict)
        elif command == 'deactivate_remap':
            self.handle_deactivate_remap()
        elif command == 'publish_goal':
            # New command to trigger publishing the goal again
            if self.remap_active and self.remapped_goal is not None:
                self.publish_goal()
            else:
                self.get_logger().warn('Cannot publish: No active remapped goal')
        else:
            self.get_logger().warning(f'Unknown command: {command}')
    
    def publish_goal(self):
        """Publish the remapped goal once"""
        if self.remapped_goal is not None:
            # Update the timestamp before publishing
            self.remapped_goal.header.stamp = self.get_clock().now().to_msg()
            self.remapped_goal_pub.publish(self.remapped_goal)
            self.get_logger().info('Published goal on demand')
            self.goal_published = True
    
    def handle_activate_remap(self, msg_dict):
        """Activate goal remapping with new goal"""
        if 'goal' not in msg_dict:
            self.get_logger().error('Activate command missing goal data')
            return
        
        goal_data = msg_dict['goal']
        
        # Create PoseStamped message
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        
        # Fill position data
        if 'position' in goal_data:
            goal_msg.pose.position.x = float(goal_data['position'].get('x', 0.0))
            goal_msg.pose.position.y = float(goal_data['position'].get('y', 0.0))
            goal_msg.pose.position.z = float(goal_data['position'].get('z', 0.0))
        
        # Fill orientation data
        if 'orientation' in goal_data:
            goal_msg.pose.orientation.x = float(goal_data['orientation'].get('x', 0.0))
            goal_msg.pose.orientation.y = float(goal_data['orientation'].get('y', 0.0))
            goal_msg.pose.orientation.z = float(goal_data['orientation'].get('z', 0.0))
            goal_msg.pose.orientation.w = float(goal_data['orientation'].get('w', 1.0))
        
        # Set remapping state
        self.remapped_goal = goal_msg
        self.remap_active = True
        self.goal_published = False  # Reset the published flag
        
        # Publish immediately
        self.publish_goal()
        self.get_logger().info('Goal remapping activated and published once')
    
    def handle_deactivate_remap(self):
        """Deactivate goal remapping"""
        self.remap_active = False
        self.remapped_goal = None
        self.goal_published = False
        self.get_logger().info('Goal remapping deactivated')
    
    def destroy_node(self):
        """Clean up resources gracefully"""
        try:
            self._shutdown_requested = True
            if hasattr(self, 'mqtt_client'):
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
        except Exception:
            pass  # Suppress any cleanup errors
        super().destroy_node()

    def heading_to_quaternion(self, heading_deg):
        """Convert heading in degrees to a ROS Quaternion (yaw only)"""
        yaw = math.radians(heading_deg)
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def signal_handler(signum, frame):
    """Handle shutdown signals gracefully"""
    # Don't print anything, just exit
    os._exit(0)


def main():
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    original_stderr = sys.stderr
    
    try:
        rclpy.init()
        node = GoalRemapper()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Don't log anything for keyboard interrupt
    except Exception:
        pass  # Suppress any other exceptions during shutdown
    finally:
        try:
            # Redirect stderr to suppress RCL shutdown errors
            sys.stderr = open(os.devnull, 'w')
            
            if 'node' in locals():
                node.destroy_node()
            
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass  # Suppress all shutdown-related errors
        finally:
            sys.stderr = original_stderr


if __name__ == '__main__':
    main()