from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'yolov5'

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
    maintainer='ikhw',
    maintainer_email='ikhwanulabiyu@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detectR = yolov5.detectR:main',
            'detectL = yolov5.detectL:main',
            'republisherR = yolov5.republisherR:main',
            'republisherL = yolov5.republisherL:main',
            'localization = yolov5.localization:main'
        ],
    },
)
