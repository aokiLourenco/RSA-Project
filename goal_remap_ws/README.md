aokiLourenco: Ok, now I want to make another program like this one provided but in python and for a different ros topic which is `/planning/mission_planning/goal`.

Help me giving the step by step for it and **when you provide code identify the file name that I need to change**.

GitHub Copilot: # Creating a Goal Remapper in Python: Files to Modify

I'll guide you through adapting the existing remapper to work with the `/planning/mission_planning/goal` topic using Python. Here are the specific files you'll need to create or modify:

## 1. Create Package Structure

First, create a new Python package:

```bash
mkdir -p ~/goal_remap/src
cd ~/goal_remap/src
ros2 pkg create --build-type ament_python goal_remap --dependencies rclpy std_msgs geometry_msgs
```

## 2. Python Implementation Files

### Main Node File
```python
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
        
        # Connect to MQTT broker
        try:
            self.get_logger().info(f"Connecting to MQTT broker at {mqtt_broker}:{mqtt_port}")
            self.mqtt_client.connect(mqtt_broker, mqtt_port, 60)
            self.mqtt_client.subscribe(mqtt_topic)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT broker: {str(e)}")
        
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
        
        # If remapping is active, use the remapped goal
        if self.remap_active and self.remapped_goal is not None:
            self.remapped_goal_pub.publish(self.remapped_goal)
            self.get_logger().info('Published remapped goal')
        else:
            # Otherwise pass through the original
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
        else:
            self.get_logger().warning(f'Unknown command: {command}')
    
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
        
        # Publish immediately
        self.remapped_goal_pub.publish(self.remapped_goal)
        self.get_logger().info('Goal remapping activated')
    
    def handle_deactivate_remap(self):
        """Deactivate goal remapping"""
        self.remap_active = False
        self.remapped_goal = None
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
```

### Package Initialization
```python
# Package initialization
```

## 3. Package Configuration Files

### Setup.py
```python
from setuptools import setup
import os
from glob import glob

package_name = 'goal_remap'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools', 'paho-mqtt'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Autoware goal remapper with MQTT triggers',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goal_remapper = goal_remap.goal_remapper:main',
        ],
    },
)
```

### Package.xml
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>goal_remap</name>
  <version>0.1.0</version>
  <description>Autoware goal remapper with MQTT triggers</description>
  <maintainer email="user@example.com">user</maintainer>
  <license>Apache License 2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## 4. Launch and Configuration Files

### Launch File
```python
emap/src/goal_remap/launch/goal_remap.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('goal_remap')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')
    
    # Declare launch arguments
    mqtt_broker = LaunchConfiguration('mqtt_broker')
    mqtt_broker_arg = DeclareLaunchArgument(
        'mqtt_broker',
        default_value='localhost',
        description='MQTT broker address'
    )
    
    mqtt_port = LaunchConfiguration('mqtt_port')
    mqtt_port_arg = DeclareLaunchArgument(
        'mqtt_port',
        default_value='1883',
        description='MQTT broker port'
    )
    
    # Create node
    goal_remapper_node = Node(
        package='goal_remap',
        executable='goal_remapper',
        name='goal_remapper',
        parameters=[
            params_file,
            {'mqtt_broker_address': mqtt_broker,
             'mqtt_broker_port': mqtt_port}
        ],
        output='screen'
    )
    
    return LaunchDescription([
        mqtt_broker_arg,
        mqtt_port_arg,
        goal_remapper_node
    ])
```

### Parameters File
```yaml
/**:
  ros__parameters:
    mqtt_broker_address: 'localhost'
    mqtt_broker_port: 1883
    mqtt_topic: 'autoware/goal_remap'
```

## 5. Create Resource Directory

```bash
mkdir -p ~/goal_remap/src/goal_remap/resource/goal_remap
touch ~/goal_remap/src/goal_remap/resource/goal_remap/placeholder
```

## 6. Create Directory Structure for Launch and Config

```bash
mkdir -p ~/goal_remap/src/goal_remap/launch
mkdir -p ~/goal_remap/src/goal_remap/config
```

## 7. Build and Run Scripts

### Build Script
```bash
#!/bin/bash

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build the workspace
cd $(dirname "$0")
colcon build --symlink-install --packages-select goal_remap
```

### Run Script
```bash
#!/bin/bash

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Source the built workspace
source $(dirname "$0")/install/setup.bash

# Launch the node
ros2 launch goal_remap goal_remap.launch.py mqtt_broker:=localhost mqtt_port:=1883
```

## 8. Make Scripts Executable

```bash
chmod +x ~/goal_remap/build.bash
chmod +x ~/goal_remap/run.bash
```

## 9. Build and Run

```bash
cd ~/goal_remap
./build.bash
./run.bash
```

## 10. Testing with MQTT Message

You can test your implementation by publishing an MQTT message with this format:

```bash
mosquitto_pub -h localhost -p 1883 -t "autoware/goal_remap" -m '{
  "command": "activate_remap",
  "goal": {
    "position": {
      "x": 10.5,
      "y": 20.3,
      "z": 0.0
    },
    "orientation": {
      "x": 0.0, 
      "y": 0.0,
      "z": 0.7071, 
      "w": 0.7071
    }
  }
}'
```

To deactivate:

```bash
mosquitto_pub -h localhost -p 1883 -t "autoware/goal_remap" -m '{
  "command": "deactivate_remap"
}'
```

This implementation will intercept and potentially modify the `/planning/mission_planning/goal` topic based on MQTT messages.

Similar code found with 5 license types

aokiLourenco: [INFO] [launch]: All log files can be found below /aoki/.ros/log/2025-05-19-19-07-24-710644-BlackBird-21021
[INFO] [launch]: Default logging verbosity is set to INFO
[WARNING] [launch_ros.actions.node]: Parameter file path is not a file: /aoki/RSA/goal_remap_ws/src/goal_remap/install/goal_remap/share/goal_remap/config/params.yaml
[INFO] [goal_remapper-1]: process started with pid [21022]
[goal_remapper-1] [INFO] [1747678045.184001401] [goal_remapper]: Connecting to MQTT broker at localhost:1883
[goal_remapper-1] [ERROR] [1747678045.186607868] [goal_remapper]: Failed to connect to MQTT broker: [Errno 111] Connection refused
[goal_remapper-1] [INFO] [1747678045.186799592] [goal_remapper]: Goal Remapper node initialized

GitHub Copilot: # Fixing the Connection Issues

I see two problems in your error output:

1. **MQTT Connection Refusal**: The broker isn't running or accessible
2. **Missing parameters file**: The parameters file path is incorrect

Let's fix both issues:

## 1. MQTT Broker Connection

First, you need to install and start the MQTT broker:

```bash
sudo apt update
sudo apt install mosquitto mosquitto-clients
sudo systemctl start mosquitto
sudo systemctl enable mosquitto
```

## 2. Fix Parameters File Path

The warning shows your workspace is at `/aoki/RSA/goal_remap_ws/` but we created files for `~/goal_remap/`. Let's fix the directory structure:

```bash
# Create the config directory in the correct location
mkdir -p /aoki/RSA/goal_remap_ws/src/goal_remap/config
```

Now create the parameters file:

```yaml
/**:
  ros__parameters:
    mqtt_broker_address: 'localhost'
    mqtt_broker_port: 1883
    mqtt_topic: 'autoware/goal_remap'
```

## 3. Update setup.py to Ensure Config Files Are Installed

Make sure your setup.py correctly includes the config files:

```python
# ...existing code...
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
# ...existing code...
```

## 4. Modify Goal Remapper to Handle MQTT Connection Failures Gracefully

```python
A/goal_remap_ws/src/goal_remap/goal_remap/goal_remapper.py
# ...existing code...

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
        
# ...existing code...
```

## 5. Rebuild Your Package

```bash
cd /aoki/RSA/goal_remap_ws
colcon build --symlink-install --packages-select goal_remap
```

## 6. Verify MQTT Is Running

Before restarting your node, check if Mosquitto is running:

```bash
systemctl status mosquitto
```

If it's not running, you may need to configure it to accept connections on localhost:

```bash
# Edit the mosquitto config file
sudo nano /etc/mosquitto/mosquitto.conf

# Add these lines
listener 1883
allow_anonymous true

# Restart mosquitto
sudo systemctl restart mosquitto
```

Now try running your node again with `./run.bash` and it should connect successfully.

Similar code found with 2 license types

aokiLourenco: sudo systemctl start mosquitto
System has not been booted with systemd as init system (PID 1). Can't operate.
Failed to connect to bus: Host is down

GitHub Copilot: # Running MQTT Broker Without Systemd

It looks like you're running in a container or environment without systemd (common in Docker containers or WSL1). Let's use direct commands to run Mosquitto instead:

## 1. Install Mosquitto if needed:

```bash
sudo apt update
sudo apt install -y mosquitto mosquitto-clients
```

## 2. Start Mosquitto directly:

```bash
# Create a simple config file
echo "listener 1883
allow_anonymous true" > /tmp/mosquitto.conf

# Run mosquitto in the background
mosquitto -c /tmp/mosquitto.conf -d
```

## 3. Verify MQTT broker is running:

```bash
# Check if mosquitto is running
ps aux | grep mosquitto

# Test publishing a message
mosquitto_pub -h localhost -p 1883 -t "test" -m "hello"

# In another terminal, subscribe to see if it works
mosquitto_sub -h localhost -p 1883 -t "test"
```

## 4. Restart your node:

```bash
cd /aoki/RSA/goal_remap_ws
source install/setup.bash
ros2 launch goal_remap goal_remap.launch.py
```

If Mosquitto still fails to start, you can modify your node to work even without MQTT by making the MQTT connection optional:

```python
# In your __init__ method:
self.mqtt_enabled = False  # Default to disabled
try:
    # Try to create MQTT client
    self.mqtt_client = mqtt.Client()
    self.mqtt_client.on_connect = self.on_connect
    self.mqtt_client.on_message = self.on_message
    
    self.get_logger().info(f"Connecting to MQTT broker at {mqtt_broker}:{mqtt_port}")
    self.mqtt_client.connect(mqtt_broker, mqtt_port, 60)
    self.mqtt_client.subscribe(mqtt_topic)
    self.mqtt_client.loop_start()
    self.mqtt_enabled = True
    self.get_logger().info("MQTT connection successful")
except Exception as e:
    self.get_logger().error(f"Failed to connect to MQTT broker: {str(e)}")
    self.get_logger().warn("Running without MQTT functionality. Manual goal remapping only.")
```

This way, your node will continue to function even without MQTT connectivity, and you could add alternative methods to control remapping (like ROS2 services/actions).

aokiLourenco: ```
header:
  stamp:
    sec: 1747664041
    nanosec: 780039798
  frame_id: map
pose:
  position:
    x: 31.0156192779541
    y: -77.63153839111328
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: -0.3959500384824782
    w: 0.9182720550173157
---
```

How can I test with these positions

GitHub Copilot: # Testing with Specific Goal Position

You can test with these specific positions in two ways:

## 1. Using MQTT to Trigger a Remapped Goal

Create a JSON file with the exact coordinates:

```bash
cat > test_goal.json << 'EOF'
{
  "command": "activate_remap",
  "goal": {
    "position": {
      "x": 31.0156192779541,
      "y": -77.63153839111328,
      "z": 0.0
    },
    "orientation": {
      "x": 0.0,
      "y": 0.0,
      "z": -0.3959500384824782,
      "w": 0.9182720550173157
    }
  }
}
EOF
```

Then publish it to your MQTT topic:

```bash
mosquitto_pub -h localhost -p 1883 -t "autoware/goal_remap" -f test_goal.json
```

## 2. Directly Publishing to ROS Topic for Testing

Create a Python script to publish directly to the goal topic:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('test_goal_publisher')
        self.publisher = self.create_publisher(
            PoseStamped, 
            '/planning/mission_planning/goal', 
            10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Goal publisher started')
        
    def timer_callback(self):
        msg = PoseStamped()
        
        # Set header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # Set position
        msg.pose.position.x = 31.0156192779541
        msg.pose.position.y = -77.63153839111328
        msg.pose.position.z = 0.0
        
        # Set orientation
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = -0.3959500384824782
        msg.pose.orientation.w = 0.9182720550173157
        
        # Publish message
        self.publisher.publish(msg)
        self.get_logger().info('Published test goal')
        

def main():
    rclpy.init()
    node = GoalPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Make it executable and run:

```bash
chmod +x /aoki/RSA/goal_remap_ws/test_publish_goal.py
cd /aoki/RSA/goal_remap_ws
source install/setup.bash
python3 test_publish_goal.py
```

## 3. Using ROS2 Command Line to Send a Single Message

```bash
cd /aoki/RSA/goal_remap_ws
source install/setup.bash
ros2 topic pub --once /planning/mission_planning/goal geometry_msgs/msg/PoseStamped "{
  header: {
    stamp: {sec: $(date +%s), nanosec: 0},
    frame_id: 'map'
  },
  pose: {
    position: {x: 31.0156192779541, y: -77.63153839111328, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: -0.3959500384824782, w: 0.9182720550173157}
  }
}"
```

These methods let you test both the direct publishing functionality and the remapping functionality with the exact coordinates you provided.

aokiLourenco: I want to it to publish only once and wait for another publishes

GitHub Copilot: # Modifying Goal Remapper to Publish Only Once

You need to modify your code to publish the goal only once when a command is received, rather than continuously. Here's how to update your goal_remapper.py:

```python
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
```

## Key Changes Made:

1. **Added a `goal_published` flag** to track if the goal was already published
2. **Modified `goal_callback`** to only pass through original goals but not republish remapped goals
3. **Added a new `publish_goal()` method** to handle the actual publishing
4. **Added a new MQTT command `publish_goal`** to trigger publishing the goal again when needed

## Testing with MQTT Commands:

1. **First publish with coordinates**:
```bash
mosquitto_pub -h localhost -p 1883 -t "autoware/goal_remap" -m '{
  "command": "activate_remap",
  "goal": {
    "position": {
      "x": 31.0156192779541,
      "y": -77.63153839111328,
      "z": 0.0
    },
    "orientation": {
      "x": 0.0,
      "y": 0.0, 
      "z": -0.3959500384824782,
      "w": 0.9182720550173157
    }
  }
}'
```

2. **To publish again** (same goal, new timestamp):
```bash
mosquitto_pub -h localhost -p 1883 -t "autoware/goal_remap" -m '{
  "command": "publish_goal"
}'
```

3. **To deactivate remapping**:
```bash
mosquitto_pub -h localhost -p 1883 -t "autoware/goal_remap" -m '{
  "command": "deactivate_remap"
}'
```

This implementation ensures the goal is published exactly once when received via MQTT, and additional publications require explicit commands.