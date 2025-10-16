import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'fish_sensor_pkg'
    
    params_file = os.path.join(
        get_package_share_directory(pkg_name),
        'params',
        'fish_sensor_params.yaml'
    )

    return LaunchDescription([
        Node(
            package=pkg_name,
            executable='fish_sensor_node',
            name='fish_sensor_node',
            output='screen',
            emulate_tty=True,
            parameters=[params_file]
        )
    ])