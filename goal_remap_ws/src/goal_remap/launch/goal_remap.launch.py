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