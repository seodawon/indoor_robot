from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    # 경로 설정 (각 패키지의 launch 파일 경로)
    ydlidar_launch_path = os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch', 'ydlidar_launch.py')
    tf_start_launch_path = os.path.join(get_package_share_directory('my_tf_ros2_test_pkg'), 'launch', 'tf_start.launch.py')
    cartographer_launch_path = os.path.join(get_package_share_directory('cartographer'), 'launch', 'cartographer_launch.py')
    sensorfusion_launch_path = os.path.join(get_package_share_directory('sensorfusion'), 'launch', 'sensorfusion.launch.py')
    # use_sim_time = LaunchConfiguration('use_sim_time', default='False')


    return LaunchDescription([
        # sensorfusion 실행
        # ydlidar_launch.py 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ydlidar_launch_path),
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(sensorfusion_launch_path),
        # ),

        ExecuteProcess(
            cmd=['ros2', 'run', 'sensorfusion', 'sensorfusion'],
            output='screen'
        ),

        # tf_start.launch.py 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tf_start_launch_path),
            
            ),

        # cartographer_launch.py 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cartographer_launch_path),
            launch_arguments={'open_rviz': 'true', 'use_sim_time': 'true'}.items()
        )
    ])