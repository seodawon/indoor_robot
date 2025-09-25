#!/bin/bash

echo "follow.sh"

# follow waypoints 경로 생성 Launch
taskset -c 12 ros2 run my_tf_ros2_test_pkg follow_waypoints &

# base_link 좌표 DB로 publish Launch
taskset -c 12 ros2 run my_tf_ros2_test_pkg base_link_publisher

wait
