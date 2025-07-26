from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    sensorfusion = Node(
        package='sensorfusion',
        executable='sensorfusion',
        name='sensorfusion',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        sensorfusion,
    ])