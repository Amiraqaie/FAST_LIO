import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    highrate_use = LaunchConfiguration('highrate')
    imu_topic = LaunchConfiguration('imu_topic')
    acc_scale = LaunchConfiguration('acc_scale')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )
    declare_highrate_cmd = DeclareLaunchArgument(
        'highrate', default_value='true',
        description='Run the IMU-rate odometry node'
    )
    declare_imu_topic_cmd = DeclareLaunchArgument(
        'imu_topic', default_value='/livox/imu',
        description='IMU topic (must match common.imu_topic in the FAST-LIO yaml)'
    )
    declare_acc_scale_cmd = DeclareLaunchArgument(
        'acc_scale', default_value='9.81',
        description='Accel scale: 9.81 for IMUs in g (Livox), 1.0 for m/s^2'
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {'use_sim_time': use_sim_time}],
        output='screen'
    )

    highrate_odom_node = Node(
        package='fast_lio',
        executable='highrate_odom',
        name='lio_highrate_odom',
        parameters=[{
            'use_sim_time': use_sim_time,
            'imu_topic': imu_topic,
            'state_topic': '/lio_state',
            'odom_topic': '/Odometry_imu',
            'frame_id': 'odom',
            'child_frame_id': 'livox_frame',
            'acc_scale': acc_scale,
            # keep these identical to mapping.* and common.* in your FAST-LIO yaml
            'gyr_cov': 0.1,
            'acc_cov': 0.1,
            'b_gyr_cov': 0.0001,
            'b_acc_cov': 0.0001,
            'time_offset_lidar_to_imu': 0.0,
        }],
        output='screen',
        condition=IfCondition(highrate_use)
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_highrate_cmd)
    ld.add_action(declare_imu_topic_cmd)
    ld.add_action(declare_acc_scale_cmd)

    ld.add_action(fast_lio_node)
    ld.add_action(highrate_odom_node)
    ld.add_action(rviz_node)

    return ld