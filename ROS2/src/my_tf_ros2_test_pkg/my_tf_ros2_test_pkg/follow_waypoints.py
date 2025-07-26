from copy import deepcopy
from geometry_msgs.msg import PoseStamped, Point
import rclpy
from std_msgs.msg import Empty
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from visualization_msgs.msg import Marker, MarkerArray
from firebase_admin import db, credentials
import firebase_admin
from rclpy.node import Node
import time
from yolo_msgs.msg import  ArrivedPoint


def parse_coordinates(data):
    data = data.strip("[]").split(", ")
    coordinates = []

    for item in data:
        lat_lon = item.split(" ")
        if len(lat_lon) != 2 or '' in lat_lon:
            print("좌표값이 없습니다") 
            continue

        try:
            lat = float(lat_lon[0]) # y
            lon = float(lat_lon[1]) # x

            # switch-case대신 딕셔너리로 좌표값에 따른 보정값 정의
            lon_adjustments = {
                -0.9227718710899353: 0.8558305576443672,# yeahgam
                -1.0520577430725098: 0.9851164296269417, # banana
                -1.042722225189209: 0.9757809117436409, # shampoo
                -1.0343279838562012: 0.9673866704106331, #mouse
                0.7631587386131287: -0.8300990510586968, #homerunball
                0.8307298421859741: -0.8976711556315422,# milk
                0.8301783800125122: -0.8971196934580803, # windbreak
                0.804151177406311: -0.8710924908518791 # daon
            }

            # lon 값에 따라 lon2 계산
            if lon in lon_adjustments:
                lon2 = float(lon + lon_adjustments[lon])
            else:
                lon2 = float(lon)

            coordinates.append(((lat, lon),(lat, lon2)))
        except ValueError as e:
            print(f"좌표 변환 실패: {e} - 원본 데이터: {lat_lon}")

    return coordinates


class Waypoints(Node):
    def __init__(self):
        super().__init__('setDBwaypoint')

        # Firebase 초기화
        cred = credentials.Certificate("/home/sh/dawon_ws/sensemart-8c5c7-firebase-adminsdk-npdg8-fccf1988f9.json")
        firebase_admin.initialize_app(cred, {
            'databaseURL': 'https://sensemart-8c5c7-default-rtdb.firebaseio.com'
        })
        # Firebase 참조 초기화
        self.ref = db.reference('/admin')
        self.state_publish_0 = self.create_publisher(ArrivedPoint, '/way_state_0',10) # 경유지 도착시에 state를 0으로 세팅하기 위한 변수
        # 퍼블리셔 설정
        self.marker_publisher = self.create_publisher(MarkerArray, 'visualization_marker', 30)
        self.input_node = rclpy.create_node('input_listener')
        self.input_subscriber = self.input_node.create_subscription(Empty, 'input_at_waypoint/input', self.input_callback, 10)

        self.marker_array = MarkerArray()
        self.navigator = BasicNavigator()
        self.robot_route = []

        # 입력 수신 여부 및 시간 초기화
        self.input_received = False
        self.last_input_time = 0

        # 초기 위치 설정
        self.timer = self.create_timer(0.5, self.publish_data_from_firebase)
        self.object_pose = {
            (0.7631587386131287,-1.4323084354400635):"home_run_ball",
            (-0.9227718710899353,-1.406002163887024):"yeah_gam",
            (0.8307298421859741,-2.5823084354400635):"good_milk",
            (-1.0520577430725098,-2.626002163887024):"banana",
            (0.8301783800125122,-3.7223084354400635):"wind_break",
            (-1.042722225189209,-3.756002163887024): "shampoo",
            (0.804151177406311,-4.852308435440064):"daon",
            (-1.0343279838562012,-4.896002163887024):"mouse",
            (0.0,0.0):"bracelet",
        } # 값을 뽑아내기 쉽게 키 값과 value 값의 위치 change
        self.set_initial_pose()
        
        # 경유지 도착시에 lat,lon 값에 해당하는 상품명을 담아서 보낼 수 있도록함
        # 데이터 주기적으로 가져오기

    def set_initial_pose(self):
        # initial_pose = PoseStamped()
        # initial_pose.header.frame_id = 'map'
        # initial_pose.header.stamp = self.get_clock().now().to_msg()
        # initial_pose.pose.position.x = 3.45
        # initial_pose.pose.position.y = -4.0
        # initial_pose.pose.orientation.z = 0.0
        # initial_pose.pose.orientation.w = 1.0

        self.navigator.waitUntilNav2Active()
        # self.navigator.setInitialPose(initial_pose)

    def input_callback(self, msg):
        self.input_received = True
        self.last_input_time = self.get_clock().now().nanoseconds  # 현재 시간 저장
        # print("Input received.")

    def publish_data_from_firebase(self):
        marker_data = self.ref.child('marker/waypoints').get()

        self.marker_array.markers.clear()

        if marker_data:
            coordinates = parse_coordinates(marker_data)
            self.update_visualization_markers(coordinates)

            if coordinates:
                self.robot_route = coordinates  # Firebase에서 가져온 좌표를 경로로 설정
                self.navigate_route()
        else:
            self.get_logger().info("좌표값 없음. 경로 생성 불가. 대기.")

    def update_visualization_markers(self, coordinates):
        self.marker_array.markers.clear()

        for i, ((lat, lon),(lat, lon2)) in enumerate(coordinates):
            visual_marker = Marker()
            visual_marker.header.frame_id = "map"
            visual_marker.header.stamp = self.get_clock().now().to_msg()
            visual_marker.ns = "coordinates"
            visual_marker.id = i
            visual_marker.type = Marker.SPHERE
            visual_marker.action = Marker.ADD

            visual_marker.pose.position = Point(x=lon2, y=lat, z=0.0)
            visual_marker.scale.x = 0.5
            visual_marker.scale.y = 0.5
            visual_marker.scale.z = 0.5
            visual_marker.color.r = 0.0
            visual_marker.color.g = 1.0
            visual_marker.color.b = 0.0
            visual_marker.color.a = 1.0

            self.marker_array.markers.append(visual_marker)

            # 데이터 로깅
            data_string = f"[{lat} {lon2}]"
            self.get_logger().info(data_string)

        self.marker_publisher.publish(self.marker_array)


    def navigate_route(self):
        for idx, ((lat, lon),(lat, lon2)) in enumerate(self.robot_route):# (lat,lon) -> 실제 물품 좌표 / (lat,lon2) -> 복도 중앙으로 재조정한 좌표
            print(f"Moving to waypoint {idx + 1}/{len(self.robot_route)}...")

            # 이동할 Pose 설정
            waypoint = PoseStamped()
            waypoint.header.frame_id = 'map'
            waypoint.header.stamp = self.get_clock().now().to_msg()
            waypoint.pose.position.x = lon2
            waypoint.pose.position.y = lat
            waypoint.pose.orientation.z = 0.0
            waypoint.pose.orientation.w = 1.0

            # 이동 시작
            self.navigator.goToPose(waypoint)

            # 이동 중 입력 확인 및 처리
            while True:
                rclpy.spin_once(self.input_node, timeout_sec=0.1)

                current_time = self.get_clock().now().nanoseconds

                # 작업이 완료되었는지 확인
                if self.navigator.isTaskComplete():
                    # 경유지 도착 때마다 출력
                    print(f"Reached waypoint {idx + 1}/{len(self.robot_route)}. Waiting for input...")
                    state_pub = ArrivedPoint() #  경유지 도착시에  state 를 0 으로 세팅하기 위한 것
                    state_pub.state = False
                    state_pub.goods = self.object_pose.get((lon,lat),None) #x,y 
                    self.state_publish_0.publish(state_pub)
                    data_string = f"[{lat} {lon}]"
                    waypointDB = db.reference('/admin/marker/arrived')
                    waypointDB.set(data_string)
                    self.get_logger().info("Data added to Firebase!")
                    time.sleep(12)
                    # 3초 대기 로직 구현
                    # wait_start_time = current_time
                    # while (current_time - wait_start_time) < 10.0 * 1e9:  # 3초 대기
                    #     rclpy.spin_once(self.input_node, timeout_sec=0.1)
                    #     current_time = self.get_clock().now().nanoseconds
                       
                        # 입력이 수신되면 정지
                        # if self.input_received:
                        #     print(f"Input received at waypoint {idx + 1}. Stopping navigation.")
                        #     break

                    if self.input_received:
                        break  # 입력을 받으면 루프 종료

                    continue  # 입력을 기다리기 위해 루프를 계속 돌기

                # 입력이 수신되었고 입력이 없어진 경우
                if self.input_received and (current_time - self.last_input_time) > 1.0 * 1e9:  # 1초
                    print(f"Input lost. Stopping at waypoint {idx + 1}.")
                    self.navigator.cancelTask()
                    self.input_received = False  # 입력이 없으므로 False로 설정

                # 입력이 수신되지 않으면 즉시 작업 취소
                if not self.input_received:
                    print(f"No input detected. Cancelling task at waypoint {idx + 1}.")
                    self.navigator.cancelTask()

                    # 입력을 기다림
                    while not self.input_received:
                        # print("Waiting for input to resume navigation...")
                        rclpy.spin_once(self.input_node, timeout_sec=0.1)

                    # print("Input received. Resuming navigation...")
                    self.navigator.goToPose(waypoint)  # 현재 경유지로 다시 이동 요청

        # time.sleep(2.0)
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print("Inspection complete!")
           
            # clearValue = f"[{-999999999} {-999999999}]"

            # waypointDB = db.reference('/admin/marker/arrived')
            # waypointDB.set(clearValue)

        elif result == TaskResult.CANCELED:
            print("Inspection was canceled.")
        elif result == TaskResult.FAILED:
            print("Inspection failed!")

        exit(0)




def main(args=None):
    rclpy.init(args=args)
    node = Waypoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()