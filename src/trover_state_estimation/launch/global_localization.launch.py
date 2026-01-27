import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_state_estimation = get_package_share_directory('trover_state_estimation')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')

    lifecycle_nodes = ['map_server', 'amcl']

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true'
    )

    # Path to your saved map (save from SLAM using: ros2 run nav2_map_server map_saver_cli -f ~/map)
    map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_state_estimation, 'maps', 'map.yaml'),
        description='Full path to map yaml file'
    )

    amcl_config = os.path.join(pkg_state_estimation, 'config', 'amcl.yaml')

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_yaml},
            {'use_sim_time': use_sim_time}
        ]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_config,
            {'use_sim_time': use_sim_time}
        ]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'node_names': lifecycle_nodes},
            {'use_sim_time': use_sim_time},
            {'autostart': True}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        #map_yaml_arg,
        #map_server_node,
        #amcl_node,
        lifecycle_manager_node,
    ])
