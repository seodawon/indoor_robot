from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_tf_ros2_test_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],    
    zip_safe=True,
    maintainer='sh',
    maintainer_email='sh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'clock_node = my_tf_ros2_test_pkg.clock_node:main',
            'mapToOdom = my_tf_ros2_test_pkg.mapToOdom:main',
            'odomToBase = my_tf_ros2_test_pkg.odomToBase:main',
            'marker_publisher = my_tf_ros2_test_pkg.marker_publisher:main',
            'base_link_publisher = my_tf_ros2_test_pkg.base_link_publisher:main',
            'follow_waypoints = my_tf_ros2_test_pkg.follow_waypoints:main',
        ],
    },
)
