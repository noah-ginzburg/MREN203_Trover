import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_bringup = get_package_share_directory('trover_bringup')
    pkg_description = get_package_share_directory('trover_description')
    pkg_localization = get_package_share_directory('trover_state_estimation')
    pkg_navigation = get_package_share_directory('trover_navigation')
    pkg_slam = get_package_share_directory('trover_slam')

    # Launch arguments
    slam = LaunchConfiguration("slam")
    slam_arg = DeclareLaunchArgument(
        "slam",
        default_value="true",
        description="Use SLAM for mapping. If false, use AMCL with a pre-built map."
    )

    robot_model = LaunchConfiguration("robot_model")
    robot_model_path = PathJoinSubstitution([pkg_description, 'urdf', robot_model])
    robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value='trover_description.urdf.xacro',
        description="Robot model filename in trover_description/urdf."
    )

    use_joint_state_publisher = LaunchConfiguration("use_joint_state_publisher")
    use_joint_state_publisher_arg = DeclareLaunchArgument(
        "use_joint_state_publisher",
        default_value="true",
        description="Publish default joint states for non-fixed joints when hardware does not.",
    )

    use_nav = LaunchConfiguration("use_nav")
    use_nav_arg = DeclareLaunchArgument(
        "use_nav",
        default_value="true",
        description="Launch navigation stack"
    )

    display = LaunchConfiguration("display")
    display_arg = DeclareLaunchArgument(
        "display",
        default_value="false",
        description="Launch RViz for visualization"
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro', ' ', robot_model_path]),
                value_type=str
            ),
            'use_sim_time': False
        }]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(use_joint_state_publisher),
    )

    # Serial communication (encoder ticks → odom velocities → EKF)
    serial_comm = Node(
        package='trover_communication',
        executable='trover_serial_comm',
        name='trover_comm_node',
        output='screen',
    )

    # LiDARs
    # laser_driver = Node(
    #     package="rplidar_ros",
    #     executable="rplidar_node",
    #     name="rplidar_node_1",
    #     parameters=[os.path.join(pkg_bringup, "config", "rplidar_a1_1.yaml")],
    #     output="screen"
    # )

    # Keyboard teleop
    keyboard_teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        remappings=[('cmd_vel', 'cmd_vel')],
    )

    # EKF local localization (always on)
    local_localization = IncludeLaunchDescription(
        os.path.join(pkg_localization, 'launch', 'local_localization.launch.py'),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # SLAM (if slam:=true)
    slam_toolbox = IncludeLaunchDescription(
        os.path.join(pkg_slam, 'launch', 'slam.launch.py'),
        launch_arguments={'use_sim_time': 'false'}.items(),
        condition=IfCondition(slam)
    )

    # AMCL global localization (if slam:=false)
    global_localization = IncludeLaunchDescription(
        os.path.join(pkg_localization, 'launch', 'global_localization.launch.py'),
        launch_arguments={'use_sim_time': 'false'}.items(),
        condition=UnlessCondition(slam)
    )

    # Navigation (if use_nav:=true)
    navigation = IncludeLaunchDescription(
        os.path.join(pkg_navigation, 'launch', 'navigation.launch.py'),
        launch_arguments={'use_sim_time': 'false'}.items(),
        condition=IfCondition(use_nav)
    )

    # RViz (if display:=true)
    rviz_config_path = os.path.join(pkg_description, 'rviz', 'default_nav.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(display)
    )

    return LaunchDescription([
        slam_arg,
        robot_model_arg,
        use_joint_state_publisher_arg,
        use_nav_arg,
        display_arg,
        robot_state_publisher,
        joint_state_publisher,
        serial_comm,
        keyboard_teleop,
        local_localization,
        slam_toolbox,
        global_localization,
        navigation,
        rviz,
    ])
