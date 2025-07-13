from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    camera_node = Node(
        package='yolov5',
        executable='detectR',
        name='detectR',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True}]
    )
    camera_node1 = Node(
        package='yolov5',
        executable='detectL',
        name='detectL',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True}]
    )
    fusion_node = Node(
        package='yolov5',
        executable='republisherR',
        name='republisherR',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True}]
    )
    fusion_node1 = Node(
        package='yolov5',
        executable='republisherL',
        name='republisherL',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True}]
    )
    return LaunchDescription([
        camera_node,
        camera_node1,
        fusion_node,
        fusion_node1,
        # Node(
        #     package='yolov5',
        #     executable='localization',
        #     name='localTotal',
        #     output='screen'),
    ])