import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    rplidar_composition = Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-port0',
                'frame_id': 'laser_frame',
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }]
         )
    
    laser_scan_filter = Node(
            package='articubot_one',
            executable='laser_scan_filter', 
            name='laser_scan_filter',
            parameters=[
                {'range_min': 0.15},
                {'range_max': 12.0},
                {'angle_filter_ranges_deg': [0.0, 0.0]}
                # {'angle_filter_ranges_deg': [40.0, 43.0, 137.0, 141.5, -43.0, -38.5, -141.0, -137.5]} #[40.0, 43.0], [137.0, 141.5], [-43.0, -38.5], [-141.0, -137.5]
            ],
         )
    

    return LaunchDescription([
        rplidar_composition,
        laser_scan_filter
    ])
