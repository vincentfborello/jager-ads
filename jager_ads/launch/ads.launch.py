from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='jager_ads', executable='camera_node'),
        Node(package='jager_ads', executable='vision_node'),
        Node(package='jager_ads', executable='turret_controller_node'),
        Node(package='jager_ads', executable='servo_hardware_node'),
    ])