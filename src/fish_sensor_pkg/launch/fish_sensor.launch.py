import os
import yaml 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer

def generate_launch_description():
    pkg_name = 'fish_sensor_pkg'
    share_dir = get_package_share_directory(pkg_name)
    
    
    
    
    params_file_path = os.path.join(share_dir, 'params', 'fish_sensor_params.yaml')

    with open(params_file_path, 'r') as f:
        try:
            params = yaml.safe_load(f)
            
            rod_ids = params['fish_sensor_node']['ros__parameters']['rod_ids']
        except Exception as e:
            print(f"Error loading or parsing params.yaml: {e}")
            return LaunchDescription() 

    if not rod_ids:
        print("rod_ids not found or empty in params.yaml!")
        return LaunchDescription()

    
    
    
    sensor_node = Node(
        package=pkg_name,
        executable='fish_sensor_node',
        name='fish_sensor_node',
        output='screen',
        emulate_tty=True,
        parameters=[params_file_path] 
    )

    
    
    
    
    
    nodes_to_launch = [sensor_node]
    
    for rod_id in rod_ids:
        
        raw_topic = f'/fish/imu/data_raw/rod_{rod_id}'
        oriented_topic = f'/fish/imu/data_with_orientation/rod_{rod_id}'
        
        
        final_topic = f'/fish/ctrl/imu/rod_{rod_id}' 
        
        madgwick_node_name = f'madgwick_filter_node_{rod_id}'
        gravity_node_name = f'gravity_canceler_node_{rod_id}'
        container_name = f'imu_filter_container_{rod_id}'

        
        madgwick_filter_node = Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name=madgwick_node_name,
            output='screen',
            parameters=[{
                'use_mag': False,
                'publish_tf': False,
                'world_frame': 'enu',
            }],
            remappings=[
                ('imu/data_raw', raw_topic),      
                ('imu/data', oriented_topic),     
            ]
        )

        
        gravity_canceler_node = ComposableNode(
            package='imu_tools',
            plugin='imu_tools::GravityCancelerNode',
            name=gravity_node_name,
            parameters=[{
                'publish_imu': True,
            }],
            remappings=[
                ('imu/data_with_orientation', oriented_topic), 
                ('imu/data', final_topic),                    
            ]
        )

        
        container = ComposableNodeContainer(
            name=container_name,
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                gravity_canceler_node,
            ],
            output='screen'
        )
        
        
        nodes_to_launch.append(madgwick_filter_node)
        nodes_to_launch.append(container)

    return LaunchDescription(nodes_to_launch)