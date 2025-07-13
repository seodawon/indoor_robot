from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    clock_node = Node(
        package='my_tf_ros2_test_pkg',
        executable='clock_node',
        name='clock_node',
        output='screen',
        emulate_tty=True,
    )

    mapToOdom = Node(
        package='my_tf_ros2_test_pkg',
        executable='mapToOdom',
        name='mapToOdom',
        output='screen',
        emulate_tty=True,
    )

    odomToBase = Node(
        package='my_tf_ros2_test_pkg',
        executable='odomToBase',
        name='odomToBase',
        output='screen',
        emulate_tty=True,
    )

    marker_publisher = Node(
        package='my_tf_ros2_test_pkg',
        executable='marker_publisher',
        name='marker_publisher',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True}]
    )

    base_link_publisher = Node(
        package='my_tf_ros2_test_pkg',
        executable='base_link_publisher',
        name='base_link_publisher',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True}]
    )

    follow_waypoints = Node(
        package='my_tf_ros2_test_pkg',
        executable='follow_waypoints',
        name='follow_waypoints',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True}]
    )

    static_odomToBase = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_odom_to_base_link',
        output='screen',
        emulate_tty=True,
        arguments=['0', '0', '0.0', '0', '0', '0', 'odom', 'base_link']
    )

    static_baseTo2dLidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_base_link_to_laser_link_2D',
        output='screen',
        emulate_tty=True,
        arguments=['0.32', '0', '0.97', '0', '0', '0', 'base_link', 'laser_link_2D']
    )

    static_baseToCamera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_base_link_to_camera_link',
        output='screen',
        emulate_tty=True,
        arguments=['0', '0', '0.8', '0', '0', '0', 'base_link', 'camera_link']
    )

    static_baseTolaser_link_3D= Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_base_link_to_3d_lidar_link',
        output='screen',
        emulate_tty=True,
        arguments=['0.32', '0', '0.8', '0', '0', '0', 'base_link', 'laser_link_3D']
    )

    static_baseToImu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_base_link_to_imu_link',
        output='screen',
        emulate_tty=True,
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
    )

    return LaunchDescription([
        clock_node,
        marker_publisher,
        mapToOdom,
        # odomToBase,
        static_odomToBase,
        static_baseTo2dLidar,
        # static_baseToCamera,
        static_baseToImu,
        static_baseTolaser_link_3D,
        # base_link_publisher,
        # follow_waypoints
    ])
