#!/bin/bash

echo "bringup.sh"

# 2D LiDAR 실행 파일
taskset -c 0 ros2 launch ydlidar_ros2_driver ydlidar_launch.py use_sim_time:=true &

# imu 실행
ros2 topic pub -r 10 /imu sensor_msgs/Imu '{orientation: {x: 0.0, y: 0.0, z: -0.7071, w: 0.7071}, angular_velocity: {x: 0.0, y: 0.0, z: 0.0}, linear_acceleration: {x: 0.0, y: 0.0, z: 0.0}}' &

# 3D_LiDAR 실행 파일
#taskset -c 1,2,3 ros2 launch cyglidar_d1_ros2 cyglidar.launch.py use_sim_time:=true &

# camera 실행 파일
#taskset -c 2,3  ros2 launch realsense2_camera rs_launch.py depth_module.profile:=1280x720x30 pointcloud.enable:=true serial_no:="'231522072849'" &

# TF 정의 및 시리얼 포트를 통한 /cmd_vel 값 발행 파일 
taskset -c 4 ros2 launch my_tf_ros2_test_pkg tf_start.launch.py

wait 


