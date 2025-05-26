# Create package from scratch
```bash
cd goal_remapper_ws # empty directory that already exists

ros2 pkg create goal_remapper \
  --build-type ament_python \
  --dependencies rclpy geometry_msgs std_msgs

touch goal_remapper/goal_remapper/goal_remapper_node.py # python code with program that does things
```

## Edit setup.py

```python
from setuptools import setup

package_name = 'goal_remapper'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/goal_remapper_launch.py']),
    ],
    install_requires=[
        'setuptools',
        'paho-mqtt',
    ],
    zip_safe=True,
    maintainer='Aoki',
    maintainer_email='ander5loure@gmail.com',
    description='MQTT-based goal remapper for Autoware',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goal_remapper_node = goal_remapper.goal_remapper_node:main',
        ],
    },
)
```

## Edit package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>goal_remapper</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="ander5loure@gmail.com">pixkit</maintainer>
  <license>TODO: License declaration</license>

  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>

```

## Create launch file

```bash
mkdir -p goal_remapper/launch
touch goal_remapper/launch/goal_remapper_launch.py
```

### goal_remapper_launch.py:
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mqtt_broker_address', default_value='localhost'),
        DeclareLaunchArgument('mqtt_broker_port', default_value='1883'),
        DeclareLaunchArgument('mqtt_topic', default_value='autoware/goal_remap'),

        Node(
            package='goal_remapper',
            executable='goal_remapper_node',
            name='goal_remapper',
            output='screen',
            parameters=[{
                'mqtt_broker_address': LaunchConfiguration('mqtt_broker_address'),
                'mqtt_broker_port': LaunchConfiguration('mqtt_broker_port'),
                'mqtt_topic': LaunchConfiguration('mqtt_topic'),
                'map_origin.latitude': 40.634646, # origins for my especific map
                'map_origin.longitude': -8.659986, # origins for my especific map
                'map_origin.elevation': 0.0
            }]
        )
    ])

```

```kotlin
goal_remapper/
├── goal_remapper/
│   ├── __init__.py
│   └── goal_remapper_node.py
├── launch/
│   └── goal_remapper_launch.py     ← this is your launch file
├── package.xml
└── setup.py
```
---

# Build and Run

```
cd ~/ros2_ws
colcon build --packages-select goal_remapper
source install/setup.bash

ros2 launch goal_remapper goal_remapper_launch.py mqtt
```

## For MQTT side

Localy on another terminal with mosquitto service running (check with `htop --filter=mosquitto` if not run with `systemctl start mosquitto`):
```
mosquitto_pub -h localhost -p 1883 -t "autoware/goal_remap" -f DENMexample.json
```

For connection between machines make sure mosquitto is not running (check with `htop --filter=mosquitto` if running kill the process PID with `kill <PID>`) and run:
```
cd goal_remapper_ws
mosquitto -c mosquitto.conf
```