import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    
    config = os.path.join(
        get_package_share_directory('sensor_pointcloud'),
        'config',
        'sensors.yaml'
        )
        
    node=Node(
        package = 'sensor_pointcloud',
        name = 'sensor_pointcloud',
        executable = 'sensor_pointcloud',
        output="screen",
        emulate_tty=True,
        parameters = [config]
    )
    ld.add_action(node)
    return ld
