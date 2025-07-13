include "map_builder.lua"
include "trajectory_builder.lua"

options = {
    map_builder = MAP_BUILDER, -- 지도를 어떻게 생성할지 설정 (2D 또는 3D SLAM 사용 결정)
    trajectory_builder = TRAJECTORY_BUILDER, -- 로봇의 이동 경로를 어떻게 생성할지 설정 (센서 데이터 기반)
    map_frame = "map", -- SLAM에서 생성된 지도가 위치할 기준 프레임 설정 -> 다른 모든 데이터가 이 좌표계에 맞춰 변환
    tracking_frame = "base_link",   -- 로봇의 추적 프레임 설정 (로봇의 위치 기준) -> 일반적으로 로봇의 중심점
    published_frame = "odom",    -- 최종적으로 퍼블리시할 프레임 -> 일반적으로 map_frame과 동일하게 설정
    odom_frame = "odom", -- 로봇의 위치 추정을 위한 기준 프레임 지정 -> 로봇의 누적된 이동 정보
    provide_odom_frame = false, -- Cartographer가 자체적으로 odom 프레임을 생성할지 여부 (false: 외부에서 제공되는 'odom' 데이터 설정)
    publish_frame_projected_to_2d = true, -- 3D 포즈 데이터를 2D 평면에 투영하여 퍼블리시 할지 여부 설정 -> 3D SLAM에서 2D 데이터를 단순화해야 할 때 유용 
    use_odometry = false, -- 외부 odometry 데이터 사용 여부
    use_nav_sat = false, -- GPS나 다른 위성 데이터 사용 여부
    use_landmarks = false,  -- 랜드마크 기반의 위치 추정 사용 여부

    num_laser_scans = 1,  -- 2D LiDAR 데이터를 사용할지 여부
    num_multi_echo_laser_scans = 0, -- 다중 에코 2D LiDAR 데이터를 사용할지 여부
    num_subdivisions_per_laser_scan = 1, -- 2D SLAM 사용할 때 파라미터
    num_point_clouds = 0,-- 3D 포인트 클라우드 데이터를 몇 개의 센서에서 받을지 설정

    lookup_transform_timeout_sec = 1.,  -- 좌표 변환을 조회할 때의 타임아웃 설정 -> 서로 다른 좌표계 간의 변환을 조회할 때 일정 시간이 걸리는데 타임아웃 시간을 초과하면 변환 조회가 실패로 처리
    submap_publish_period_sec = 0.3,  -- 서브맵(부분 지도)을 '0.1초'마다 퍼블리시
    pose_publish_period_sec = 3e-3, -- 로봇의 포즈(위치와 자세) 퍼블리시 주기 
    trajectory_publish_period_sec = 30e-3, -- 경로 데이터 퍼블리시 주기 
    
    -- 데이터 샘플링 비율 '1.0'은 모든 데이터 사용, '0.5'는 데이터의 절반 사용 --
    rangefinder_sampling_ratio = 1., -- LiDAR 같은 거리 측정 장치 데이터의 샘플링 비율
    odometry_sampling_ratio = 1., -- Odometry 데이터 샘플링 비율
    fixed_frame_pose_sampling_ratio = 1., -- 고정 프레임 위치 데이터의 샘플링 비율
    imu_sampling_ratio = 1., -- IMU 데이터의 샘플링 비율 
    landmarks_sampling_ratio = 1., -- 랜드마크 데이터의 샘플링 비율
}

MAP_BUILDER.use_trajectory_builder_2d = true --  SLAM 시스템이 2D 경로 작성기를 사용하도록 지정

TRAJECTORY_BUILDER_2D.min_range = 0.15 --  LIDAR와 같은 센서가 인식할 수 있는 최소 거리 설정 ->  0.15미터보다 가까운 거리는 무시
TRAJECTORY_BUILDER_2D.max_range = 10.0  -- LIDAR가 인식할 수 있는 최대 거리 설정 ->  3.0미터보다 먼 거리는 무시
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3. -- 데이터가 누락된 경우 해당 광선(레이)의 길이 설정
TRAJECTORY_BUILDER_2D.use_imu_data = false -- IMU 데이터를 사용하도록 설정
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true --  온라인 상관 스캔 매칭을 사용하도록 설정 -> 실시간으로 로봇의 현재 위치를 더 정확하게 추정하는 데 사용되는 기법
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1) --  모션 필터의 최대 각도를 라디안으로 설정 -> 로봇이 특정 각도 이상 회전했을 때만 새로운 데이터를 수집
--  TEST 
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 0.01 --  Ceres 스캔 매처에서 변환(translation)에 대한 가중치를 0.01로 설정 -> 로봇의 위치 변화에 대한 중요도를 결정
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20 -- Ceres 스캔 매처에서 사용할 최대 반복 횟수 설정 ->  최적화를 수행할 때의 반복 횟수를 제한
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1 -- 누적 범위 데이터의 개수 설정 -> SLAM에서 스캔 데이터를 얼마나 자주 축적할지를 결정
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05 --  Voxel 필터의 크기를 0.05미터로 설정
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 45 -- 서브맵 하나에 포함될 범위 데이터의 개수 설정
MAP_BUILDER.num_background_threads = 4 -- 백그라운드에서 작업을 수행할 스레드의 수 설정

--  TEST
POSE_GRAPH.constraint_builder.min_score = 0.65 --  위치 그래프에서 제약 조건을 생성할 때 최소 점수 설정 -> 스캔 매칭이 유효한지 여부를 결정
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7 -- 전역 위치 추정을 할 때 필요한 최소 점수 설정 -> 지도를 처음 탐색할 때 사용하는 설정
POSE_GRAPH.global_sampling_ratio = 0.001 -- 전역 샘플링 비율 설정 -> SLAM이 전역적으로 위치를 재추정할 때 샘플링할 데이터의 비율을 결정
POSE_GRAPH.constraint_builder.sampling_ratio = 0.001 -- 제약 조건 빌더의 샘플링 비율 설정 -> 위치 그래프에서 사용할 데이터를 얼마나 자주 샘플링할지를 결정

POSE_GRAPH.optimize_every_n_nodes = 10 -- 위치 그래프를 최적화할 때마다 10개의 노드를 처리하도록 설정 --> SLAM이 경로를 얼마나 자주 최적화할지를 결정


return options