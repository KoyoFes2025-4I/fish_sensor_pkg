from launch import LaunchDescription
from launch_ros.actions import Node
import os
import yaml
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'fish_sensor_pkg'
    share_dir = get_package_share_directory(pkg_name)

    params_file_path = os.path.join(share_dir, 'params', 'fish_sensor_params.yaml')
    with open(params_file_path, 'r') as f:
        params = yaml.safe_load(f)
        rod_ids = params['fish_sensor_node']['ros__parameters']['rod_ids']

    if not rod_ids:
        print("rod_ids not found or empty in params.yaml!")
        return LaunchDescription()

    nodes_to_launch = []

    # fish_sensor_node (すべてのrodをまとめて扱う)
    nodes_to_launch.append(Node(
        package=pkg_name,
        executable='fish_sensor_node',
        name='fish_sensor_node',
        output='screen',
        emulate_tty=True,
        parameters=[params_file_path]
    ))

    # 各 rod_id ごとの madgwick + gravity_canceler を起動
    for rod_id in rod_ids:
        madgwick_node = Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name=f'madgwick_filter_node_{rod_id}',
            output='screen',
            parameters=[{
                'use_mag': False,
                'publish_tf': False,
                'world_frame': 'enu',
            }],
            remappings=[
                ('imu/data_raw', f'/fish/imu/data_raw/rod_{rod_id}'),
                ('imu/data', f'/fish/imu/data_with_orientation/rod_{rod_id}')
            ]
        )

        gravity_node = Node(
            package=pkg_name,
            executable='gravity_canceler_node',
            name=f'gravity_canceler_node_{rod_id}',
            output='screen',
            arguments=[rod_id],  # ★ これが重要！
            remappings=[
                ('/fish/imu/data_with_orientation', f'/fish/imu/data_with_orientation/rod_{rod_id}')
            ]
        )
        

        nodes_to_launch += [madgwick_node, gravity_node]

    return LaunchDescription(nodes_to_launch)
