#!/usr/bin/env python3
import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from yolo_msgs.msg import BoundingBoxArray, BoundingBox, SensorFusion,ArrivedPoint
from rclpy.qos import QoSProfile, ReliabilityPolicy,HistoryPolicy
import loguru
from std_msgs.msg import Float32,Bool
import firebase_admin
from firebase_admin import db, credentials
import math
import numpy as np
# 문제!!!! 라이다에 인식이 안돼도 객체인식된 선에 색이 변함
# 문제1 for문 많아서 딜레이 -> 3중 for에서 2중 for 로 변경
# 문제2 사람이 인식 안되면 마지막 인식된걸로 저장됨 -> 타이머써서 1초마다 초기화시켜 해결
def Rz(p):
    return np.array([
        [np.cos(p), -np.sin(p), 0],
        [np.sin(p), np.cos(p), 0],
        [0, 0, 1]
    ])

def Rx(t):
    return np.array([
        [1, 0, 0],
        [0, np.cos(t), -np.sin(t)],
        [0, np.sin(t), np.cos(t)]
    ])

def Ry(r):
    return np.array([
        [np.cos(r), 0, np.sin(r)],
        [0, 1, 0],
        [-np.sin(r), 0, np.cos(r)]
    ])

class Republisher(Node):
    def __init__(self):
        super().__init__('inference_republisher1')
        self.cv_bridge = CvBridge()
        self.img = None # 이미지 받을 곳
        self.lidar_data = np.zeros((221, 2)) # 라이다 받을 곳 #여기
        self.box = []
        self.send_data = []
        self.count = 0
        self.filter = LaserScan()
        self.value=SensorFusion()
        self.goods =str()
        self.goods = ""
        self.state = Bool() # 기본 값 true
        self.state.data = True
        cred = credentials.Certificate("/home/sh/dawon_ws/sensemart-8c5c7-firebase-adminsdk-npdg8-fccf1988f9.json")
        firebase_admin.initialize_app(cred, {
            'databaseURL': 'https://sensemart-8c5c7-default-rtdb.firebaseio.com'
        })
        # Firebase 참조 초기화
        self.ref = db.reference('/admin')
        self.state_publish_1 = self.create_publisher(Bool,'/way_state_1',10) # 팔찌를 통해서 객체 인식을 한 후에 보낸 신호
        self.state_subscribe_0 = self.create_subscription(ArrivedPoint,'/way_state_0',self.state_0_callback,10) # 경유지에 도착시에 보내는 신호
        
        self.u, self.v = None, None
        self.img_sub = self.create_subscription(Image, '/inference/image_raw1', self.img_callback, 10) # 이미지 보내줌
        # self.lidar_sub = self.create_subscription(LaserScan,'/lidar', self.lidar_callback,10)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, 
            history=HistoryPolicy.KEEP_LAST, 
            depth=10  # depth는 필요한 값으로 설정
        ) # 라이다 정책에 맞는 qos
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile)
        # self.lidar_sub = self.create_subscription(LaserScan,'/scan', self.lidar_callback,10)
        self.imgdata_sub = self.create_subscription(BoundingBoxArray, '/inference/object_raw1', self.imgdata_callback, 10) # 이미지 근본적인 정보
        self.point_data= self.create_publisher(SensorFusion,'/camera_dataL',10)
        # self.send_scan = self.create_publisher(LaserScan, '/scan_filter', 10)
        self.timer = self.create_timer(1, self.timer_callback)

        self.inc_angle = 0.5
        #logitech 거꾸로는 z-> 9.5cm x -> 0~1cm y -> 0cm logitech  정상 z-> 8.5m y-> 0cm x->-0~1cm     
        self.camera = np.array([[0.00],[0],[0.095]])  # 카메라 위치

        # 팬(pan) 각도와 틸트(tilt) 각도 (라디안 단위)
        # p = np.radians(-90)  # -60도 Z # 여기
        # t = np.radians(0)    # 0도 Y
        # roll = 0    # X회전 
        # #logitech 거꾸로는 x-> -90 z -> -90 logitech  정상 x-> 90 z -> 90
        # # R = Rz(p) @ Rx(t) @ Ry(-roll) @ Rx(-np.pi/2)  # 여기
        p = np.radians(-90)  # -90도 Z (카메라의 방향을 맞춤)
        t = np.radians(0)    # Y축 회전은 없으므로 0도
        roll = np.radians(100)          # X축 회전은 없으므로 0도

        # 회전 행렬 정의 (라이다 좌표계를 카메라 좌표계로 변환)
        R = Rz(p) @ Ry(t) @ Rx(roll)
        self.R_inv = np.linalg.inv(R) #역행렬 외부 파라미터

        # 카메라 파라미터 내부 파라미터
        self.K = np.array([  # 여기
            [868.7118249, 0, 345.08293437],
            [0, 809.2191629, 263.54213374],
            [0, 0, 1]
        ])
        
    def state_0_callback(self,msg):
        # loguru.logger.info("hi3")
        self.goods =msg.goods
        loguru.logger.info(self.goods)
        # recognize(self) # 라이다 콜백함수에 넣으면 계속 실행된다.

    def img_callback(self, msg):
        self.img = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8') # YOLO이미지 받기

    def imgdata_callback(self, msg):
        self.box.clear() # 초기화
        for bounding_box in msg.bounding_box_array:
            self.box.append((bounding_box.x_max, bounding_box.x_min, bounding_box.y_max,bounding_box.y_min,bounding_box.class_name))
            # 라벨링 네모 나옴
    def lidar_callback(self, msg):
        if self.img is not None :
            self.filter = msg
            recognize(self) # 라이다 콜백함수에 넣으면 계속 실행된다.
            for i in range (111):#169 #여기 각도만큼 반복
                project_lidar_to_image(msg.ranges[165 + i], 332.5  + self.inc_angle * i, self) # 여기
                if len(self.box) > 0:
                    if self.u is not None and self.v is not None and self.box is not None:
                        # project_lidar_to_image(msg.ranges[165 + i], 332.5  + self.inc_angle * i, self) # 여기
                        catch(self, msg.ranges[165 + i], 332.5 + self.inc_angle * i, i) #여기
                # elif len(self.box) == 0:        
                #      break
            Fusion(self.img)
            self.point_data.publish(self.value)
            # loguru.logger.info(f'detect value : {self.value.obstacle_distance: .4f}/ detect name : {str(self.value.class_name)} / {str(self.value.camera_p)}')
            self.value.obstacle_distance = 0.0
            self.value.class_name=''
            self.value.camera_p='L'
                    # self.filter.ranges[485 + i] = 1000.0# 여기
                # project_lidar_to_image(msg.ranges[625 + i], 152.5  + self.inc_angle * i, self) # 여기
            # self.send_scan.publish(self.filter)
            self.send_data.clear()

    def timer_callback(self):
        self.box.clear()

def project_lidar_to_image(distance, angle, self): # 모든 계산
    # 1. 라이다 데이터를 월드 좌표계로 변환
    world_point = lidar_to_world(distance, angle) # 맞음

    # 2. 월드 좌표계를 카메라 좌표계로 변환
    camera_point = self.R_inv @ (world_point + self.camera)

    # 3. 카메라 좌표계를 이미지 평면으로 투영
    image_point = project_to_image(camera_point, self)
    u, v = int(image_point[0]), int(image_point[1])

    # 이미지 범위 내인지 확인하고 점을 그리기
    if 0 <= u < self.img.shape[1] and 0 <= v < self.img.shape[0]:
        self.u, self.v = u, v
        cv2.circle(self.img, (u, v), 3, (0, 0, 255), -1)

def lidar_to_world(distance, angle): # 카메라 좌표계에 라이다 좌표계 적용 하기 위한 함수
    # 각도를 라디안으로 변환
    theta = np.deg2rad(angle)
    x = distance * np.cos(theta)
    y = distance * np.sin(theta)
    z = 0  # 2D 라이다 데이터는 z = 0
    return np.array([[x], [y], [z]])

def project_to_image(camera_point, self): # CAMERA를 이미지 포인트로 변환하는 과정
    # 카메라 내재 파라미터 행렬을 사용하여 투영
    image_point_homogeneous = self.K @ camera_point
    # 동차 좌표계를 2D 좌표로 변환
    image_point = image_point_homogeneous[:2] / image_point_homogeneous[2]
    return image_point

def catch(self, dis, angle, count): # 박스 안에 있어? 없어?
    for i in range(len(self.box)): # 박스 개수에 따라 여러개가 된다면? 
        x_max, x_min, y_max, y_min,class_name = self.box[i]
        if x_max >= self.u >= x_min and class_name != "bracelet": #send_data에 값을 담아서 보냄 그런데 bracelet 즉 팔찌는 안보냄
            self.send_data= [(class_name, dis)]
        if count == 110:
            if len(self.send_data) > 0 and len(self.send_data) != 0:
                self.value.class_name, self.value.obstacle_distance = self.send_data[len(self.send_data)//2]
                self.value.camera_p='L'
            # cv2.circle(self.img, (self.u, self.v), 3, (255, 0, 0), -1)
            # self.send_data.append((angle, dis))
        # else :
            # self.filter.ranges[625 + count] = 1000.0 #여기

def recognize(self):
    #next() 함수는 주어진 조건을 만족하는 첫 번째 값을 반환
    #enumerate()는 리스트의 인덱스와 값을 동시에 사용하고 싶을 때 매우 유용한 함수
    # 만족하는 것이 없으면 None을 반환하도록 설정
    if self.goods:
        # loguru.logger.info(self.box)
        index_bracelet = next((index for index, bounding_box in enumerate(self.box) if bounding_box[4] == "bracelet"), None)  # 리스트 컴프리헨션 기술 / self.box에 bracelet이 있으면 해당 index 반환
        index_goods = next((index for index, bounding_box in enumerate(self.box) if bounding_box[4] == self.goods), None)
        waypointDB = db.reference('/admin/marker/bracelet')
        loguru.logger.info(index_goods)
        loguru.logger.info("hoho")
        loguru.logger.info(index_bracelet)
        if self.count > 6:
            waypointDB.set(0)
            self.count =0
        if index_bracelet  is not None and index_goods is not None:
                if self.box[index_goods][1] < self.box[index_bracelet][1] and self.box[index_goods][0]> self.box[index_bracelet][0] or self.box[index_goods][3] < self.box[index_bracelet][3] and self.box[index_goods][2] > self.box[index_bracelet][2]:
                    loguru.logger.info("hi")
                    self.state_publish_1.publish(self.state)
                    waypointDB.set(1)
                    self.count += 1

                # 여기 팔찌 인식 박스랑 장바구니상품들 인식 박스 안에 들어오면 허용하게끔해서 state=1로 변환하도록 토픽 발행해주는 코드 작성해야함
                #goods 초기화 왜냐하면 여기서 계산되는 비용을 줄이려고 근데, 그거를 localization 파일에서 움직이기 시작할때 값을 보내면 초기화 해주도록 아니면 실행이 안되도록 해야함 -> 너무 무리되면 바꾸는 걸로 이렇게
                #가려져서 상품 객체 인식이 안될 경우 고정 네모 박스를 넣어주어야한다.
def Fusion(image): # 이미지 띄우기
    cv2.imshow("FusionL", image)
    if cv2.waitKey(1) == ord('q'):
        raise StopIteration


def main(args=None):
    rclpy.init(args=args)

    republisher = Republisher()
   
    rclpy.spin(republisher)
    
    republisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


