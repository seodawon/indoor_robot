#!/bin/bash

echo "camera.sh"

# Camera 실행 파일
taskset -c 8,9,10,11 ros2 launch yolov5 sensor_fusion.launch.py &

# Localization, IMU, Press 실행 파일 
taskset -c 7 ros2 run yolov5 localization

wait

