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

    # -------------------------------------------------------
    # ノード 1: センサーノード (自作)
    # /fish/imu/data_raw (生データ) と /fish/ctrl/out (回転) を発行
    # -------------------------------------------------------
    sensor_node = Node(
        package=pkg_name,
        executable='fish_sensor_node',
        name='fish_sensor_node',
        output='screen',
        emulate_tty=True,
        parameters=[params_file]
    )
    
    # -------------------------------------------------------
    # ノード 2: Madgwickフィルター (姿勢計算)
    # 入力: /fish/imu/data_raw
    # 出力: /fish/imu/data_with_orientation
    # -------------------------------------------------------
    madgwick_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='madgwick_filter_node',
        output='screen',
        parameters=[{
            'use_mag': False,
            'publish_tf': True,
            'world_frame': 'enu',
            'gain': 0.02,         # 姿勢変化をゆっくりに（ノイズ減）
            'zeta': 0.005,        # ジャイロドリフト補正をわずかに有効化
            'sample_freq': 100.0,
        }],
        remappings=[
            ('imu/data_raw', '/fish/imu/data_raw'),
            ('imu/data', '/fish/imu/data_with_orientation'),
        ]
    )

    # -------------------------------------------------------
    # ノード 3: 重力除去フィルター (自作ノード)
    # 入力: /fish/imu/data_with_orientation
    # 出力: /fish/ctrl/imu
    # -------------------------------------------------------
    gravity_canceler_node = Node(
        package=pkg_name,
        executable='gravity_canceler_node',
        name='gravity_canceler_node',
        output='screen',
        parameters=[{
            'publish_imu': True,
        }],
        remappings=[
            ('imu/data_in', '/fish/imu/data_with_orientation'),  # 入力
            ('imu/data_out', '/fish/ctrl/imu'),                 # 出力
        ]
    )

    return LaunchDescription([
        sensor_node,
        madgwick_filter_node,
        gravity_canceler_node
    ])
