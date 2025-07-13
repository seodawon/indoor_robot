from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 경로 설정 (각 패키지의 launch 파일 경로)
    ydlidar_launch_path = os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch', 'ydlidar_launch.py')
    tf_start_launch_path = os.path.join(get_package_share_directory('my_tf_ros2_test_pkg'), 'launch', 'tf_start.launch.py')
    sensorfusion_launch_path = os.path.join(get_package_share_directory('sensorfusion'), 'launch', 'sensorfusion.launch.py')
    cyglidar_launch_path = os.path.join(get_package_share_directory('cyglidar_d1_ros2'), 'launch', 'cyglidar.launch.py')

    # use_sim_time 설정
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        # use_sim_time 선언
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        # ydlidar 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ydlidar_launch_path),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # cyglidar 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cyglidar_launch_path),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # sensorfusion 실행
        ExecuteProcess(
            cmd=['ros2', 'run', 'sensorfusion', 'sensorfusion'],
            additional_env={'ROS_USE_SIM_TIME': use_sim_time},
            output='screen'
        ),

        # tf_transfrom, odom, clock 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tf_start_launch_path),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
