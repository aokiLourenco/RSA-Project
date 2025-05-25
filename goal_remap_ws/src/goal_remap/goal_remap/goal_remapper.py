#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import json
import paho.mqtt.client as mqtt

class GoalRemapper(Node):
    def __init__(self):
        super().__init__('goal_remapper')
        
        # Declare parameters
        self.declare_parameter('mqtt_broker_address', 'localhost')
        self.declare_parameter('mqtt_broker_port', 1883)
        self.declare_parameter('mqtt_topic', 'autoware/goal_remap')
        
        # Get parameters
        mqtt_broker = self.get_parameter('mqtt_broker_address').value
        mqtt_port = self.get_parameter('mqtt_broker_port').value
        mqtt_topic = self.get_parameter('mqtt_topic').value
        
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
            self.get_logger().info(f"Received MQTT message: {payload}")
            
            msg_dict = json.loads(payload)
            self.process_mqtt_message(msg_dict)
            
        except json.JSONDecodeError:
            self.get_logger().error(f"Failed to parse MQTT message as JSON")
        except Exception as e:
            self.get_logger().error(f"Error processing MQTT message: {str(e)}")
    
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
        if hasattr(self, 'mqtt_client'):
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        super().destroy_node()


def main():
    rclpy.init()
    node = GoalRemapper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()