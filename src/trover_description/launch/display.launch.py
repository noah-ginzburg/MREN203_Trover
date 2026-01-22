import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_description = get_package_share_directory('trover_description')
    
    urdf_path = os.path.join(pkg_description, 'urdf', 'trover_description.urdf.xacro')
    rviz_config_path = os.path.join(pkg_description, 'rviz', 'default.rviz')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    robot_description = Command(['xacro', ' ', urdf_path])

    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
        }]
    )
    
    joint_state_publisher_node = Node( #remove once the real robot publishes joint states
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path], #rviz config path
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ])