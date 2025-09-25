# 센스만점: 시각장애인을 위한 실내 쇼핑 보조 로봇

### 챗봇 연동 실내 쇼핑 보조 로봇 프로젝트
- 프로젝트 기간: 2024.04.01 ~ 2024.12.04 (8개월)  
- 참여인원: 5명

<br>

## 🎥 프로젝트 소개  
[![센스만점 Demo](https://img.youtube.com/vi/oNAtSPKoOKU/0.jpg)](https://youtu.be/oNAtSPKoOKU)
➡ 영상 클릭 시, YouTube 재생  

**센스만점**은 챗봇과 자율주행 로봇을 연동하여, 시각장애인이 대형 마트에서 원하는 상품을 쉽게 찾고 구매할 수 있도록 돕는 **실내 쇼핑 보조 시스템**입니다.  

- **ROS2, Java(Android), Python, YOLOv5, STM32** 기반  
- 2D LIDAR, 카메라, UWB를 결합한 **정밀 복합 측위 시스템**  
- **객체 인식 팔찌 + 압력 센서**를 통한 직관적 사용자 상호작용  

<br>

## 🔧 주요 기능
- 🛒 **앱 연동 최적 경로 안내**: 사용자가 앱에서 장바구니에 상품을 담으면, 로봇이 마트 내 최적 경로를 계산하고 음성으로 안내  
- ✋ **팔찌 기반 상품 확인**: 목적지 도착 후 팔찌를 상품 근처에 가져가면, 객체 인식을 통해 “찾으시는 상품이 맞습니다”라는 음성 피드백 제공  
- ⏸️ **압력 센서 기반 사용자 제어**: 카트 손잡이 압력 감지 → 주행 즉시 정지/이동 제어  
- 📍 **복합 측위(Localization) 시스템**: 2D LIDAR + 카메라 + 단일 UWB 거리 정보를 결합하여 실내 환경에서도 정확한 위치 추정  

<br>

## 🚀 전체 실행 순서
```bash
# bringup.sh
#!/bin/bash
echo "bringup.sh"

# 2D LiDAR 실행 파일
taskset -c 0 ros2 launch ydlidar_ros2_driver ydlidar_launch.py use_sim_time:=true &

# IMU 실행
ros2 topic pub -r 10 /imu sensor_msgs/Imu '{orientation: {x: 0.0, y: 0.0, z: -0.7071, w: 0.7071}, angular_velocity: {x: 0.0, y: 0.0, z: 0.0}, linear_acceleration: {x: 0.0, y: 0.0, z: 0.0}}' &

# 3D LiDAR (옵션)
# taskset -c 1,2,3 ros2 launch cyglidar_d1_ros2 cyglidar.launch.py use_sim_time:=true &

# 카메라 실행 (옵션)
# taskset -c 2,3 ros2 launch realsense2_camera rs_launch.py depth_module.profile:=1280x720x30 pointcloud.enable:=true serial_no:="'231522072849'" &

# TF 및 /cmd_vel 발행
taskset -c 4 ros2 launch my_tf_ros2_test_pkg tf_start.launch.py

wait
```
```bash
# camera.sh
#!/bin/bash
echo "camera.sh"

# Camera 실행
taskset -c 8,9,10,11 ros2 launch yolov5 sensor_fusion.launch.py &

# Localization, IMU, Press 실행
taskset -c 7 ros2 run yolov5 localization

wait
```
```bash
# follow.sh
#!/bin/bash
echo "follow.sh"

# Follow waypoints 실행
taskset -c 12 ros2 run my_tf_ros2_test_pkg follow_waypoints &

# base_link 좌표 DB publish 실행
taskset -c 12 ros2 run my_tf_ros2_test_pkg base_link_publisher

wait
```

