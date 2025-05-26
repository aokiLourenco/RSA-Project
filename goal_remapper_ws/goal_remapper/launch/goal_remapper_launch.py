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
                'map_origin.latitude': 40.634646,
                'map_origin.longitude': -8.659986,
                'map_origin.elevation': 0.0
            }]
        )
    ])
