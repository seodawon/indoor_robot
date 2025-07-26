import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32,Bool
from yolo_msgs.msg import  SensorFusion,Local,ArrivedPoint
import tf2_ros
import serial
import numpy as np
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Empty
from firebase_admin import db, credentials
import firebase_admin

class Localization(Node):
    def __init__(self):
        super().__init__('localization')
        self.imu_publisher = self.create_publisher(Imu, '/imu', 10)
        cred = credentials.Certificate("/home/sh/dawon_ws/sensemart-8c5c7-firebase-adminsdk-npdg8-fccf1988f9.json")
        firebase_admin.initialize_app(cred, {
            'databaseURL': 'https://sensemart-8c5c7-default-rtdb.firebaseio.com'
        })
        # Firebase 참조 초기화
        self.ref = db.reference('/admin')
        self.position = Local()
        self.cameraL_data = SensorFusion()
        self.cameraR_data = SensorFusion()
        self.state = Bool() # 기본 값 true
        self.state.data = True
        self.go = Bool()
        self.go.data = True
        self.waypointDB = db.reference('/admin/marker/go')
        self.bracelet = db.reference('/admin/marker/bracelet')
        # self.state_1 = Bool() # 팔찌를 통해서 객체 인식을 한 후에 보낸 신호
        # self.state_0 = Waypoint()# 경유지에 도착시에 보내는 신호
        self.state_subscribe_0 = self.create_subscription(ArrivedPoint,'/way_state_0',self.state_0_callback,10) # 경유지에 도착시에 보내는 신호
        self.state_subscribe_1 = self.create_subscription(Bool,'/way_state_1',self.state_1_callback,10) # 팔찌를 통해서 객체 인식을 한 후에 보낸 신호
        self.input_publisher =self.create_publisher(Empty, 'input_at_waypoint/input', 10)
        self.object_pose = {
            "home_run_ball":(0.7631587386131287,-1.4323084354400635),
            "yeah_gam":(-0.9227718710899353,-1.406002163887024),
            "good_milk":(0.8307298421859741,-2.5823084354400635),
            "banana":(-1.0520577430725098,-2.626002163887024),
            "shampoo":(-1.042722225189209,-3.756002163887024),
            "wind_break":(0.8301783800125122,-3.7223084354400635),
            "mouse":(-1.0343279838562012,-4.896002163887024),
            "daon":(0.804151177406311,-4.852308435440064),
            "bracelet":(0.0,0.0),
        }
        self.ser = serial.Serial('/dev/ttyUSB1', 115200)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.subscriber_R= self.create_subscription(SensorFusion, '/camera_dataR', self.cameraR_callback, 10) # 왼쪽 카메라로 부터 객체인식한 사물과의 거리 전송
        self.subscriber_L = self.create_subscription(SensorFusion, '/camera_dataL', self.cameraL_callback, 10) # 오른쪽 카메라로 부터 객체인식한 사물과의 거리 전송
        self.local_position = self.create_publisher(Local,'/local_position', 10)
        self.robot = self.create_publisher(Bool,'/robot', 10)
        
    def cameraR_callback(self,msg):
        self.cameraR_data = msg
    
    def state_0_callback(self,msg): # 경유지에 도착시에 보내는 신호
        # state_0 = ArrivedPoint()
        # state_0 = msg
        self.go.data = False # 경유지 도착시에 로봇에게 멈추라는 신호
        self.robot.publish(self.go)
        self.state.data = msg.state
    
    def state_1_callback(self,msg): # 팔찌를 통해서 객체 인식을 한 후에 보낸 신호
        state_1 = Bool()
        state_1 = msg
        self.state.data = True

    def cameraL_callback(self,msg):
        self.cameraL_data = msg
        
    def publish_message(self):
        msg = Empty()  # 빈 메시지 생성
        self.input_publisher.publish(msg)  # 메시지 발행
        self.get_logger().info('Publishing: %s' % msg)

    def timer_callback(self):
        self.ser.flushInput()
        # 시리얼 데이터 읽기
        serial_data = self.ser.readline().decode('utf-8').strip().replace('\x00', '')
        # print('a')
        parts = serial_data.split(',')
        if len(parts) != 4:
            return
        try:
            if(parts[0] == ''):
                robot_state = None
            else:
                robot_state = int(parts[0])
            yaw = float(parts[1])
            press = int(parts[2])
            range1 = float(parts[3])
            if(self.state.data and not press and not self.go.data):
               self.go.data= True
               self.waypointDB.set(1)
               self.bracelet.set(0)
               self.robot.publish(self.go)
            if (press and self.state.data and self.go.data):# 기본적으로 객체 인식 성공해서 state가 1이고 압력센서가 1 즉, 잡았을때만 실행이 되지만 경유지에 도착시에는 0으로 바뀌고, 객체 인식 후에 1로 변경
                self.publish_message()
                self.waypointDB.set(0)

            self.get_logger().info(f" check = {range1} ,{press} ,{self.state.data}, {self.go.data}")
            if  (range1 != 0.0 and self.cameraR_data.obstacle_distance*self.cameraL_data.obstacle_distance) != 0.0 and str(self.cameraL_data.class_name) != ''and str(self.cameraR_data.class_name) != '':
                self.a1 = {'x': -0.05209118127822876, 'y': -6.183788299560547, 'd': range1} # 추후에 조금 수정해야할듯 
                self.a2 = {'x': self.object_pose[str(self.cameraL_data.class_name)][0],  'y': self.object_pose[str(self.cameraL_data.class_name)][1], 'd': self.cameraL_data.obstacle_distance}
                self.a3 = {'x': self.object_pose[str(self.cameraR_data.class_name)][0],  'y': self.object_pose[str(self.cameraR_data.class_name)][1], 'd': self.cameraR_data.obstacle_distance}
                # self.position.robot_x, self.position.robot_y = self.calculate_position()
               
                if str(self.cameraR_data.class_name) == "yeah_gam" and str(self.cameraL_data.class_name) == "home_run_ball":
                    self.position.robot_x, self.position.robot_y =-0.0669413134455681,-1.406002163887024
                elif str(self.cameraR_data.class_name) == "banana" and str(self.cameraL_data.class_name) == "good_milk":
                    self.position.robot_x, self.position.robot_y = -0.0669413134455681,-2.5573084354400635
                elif str(self.cameraR_data.class_name) == "shampoo" and str(self.cameraL_data.class_name) == "wind_break":
                    self.position.robot_x, self.position.robot_y = -0.0669413134455681,-3.6823084354400635
                elif str(self.cameraR_data.class_name) == "mouse" and str(self.cameraL_data.class_name) == "daon":
                    self.position.robot_x, self.position.robot_y =-0.0669413134455681,-4.807308435440064
            else : 
                self.position.robot_x, self.position.robot_y= 0.0 , 0.0

            self.get_logger().info(f" X = {self.position.robot_x}, Y = {self.position.robot_y}")
            self.local_position.publish(self.position)

            # IMU 메시지 생성 및 설정
            imu_msg = Imu()
            imu_msg.header.frame_id = 'imu_link'
            imu_msg.header.stamp = self.get_clock().now().to_msg()

             # 가속도 및 각속도 설정
            imu_msg.linear_acceleration.x = 0.0
            imu_msg.linear_acceleration.y = 0.0
            imu_msg.linear_acceleration.z = 0.0
            imu_msg.angular_velocity.x = 0.0
            imu_msg.angular_velocity.y = 0.0
            imu_msg.angular_velocity.z = 0.0

            # yaw 보정 값 설정 (예: -90도 보정)
            yaw_offset = 100  # 90도 왼쪽으로 회전

            # 보정된 yaw 값을 쿼터니언으로 변환하여 IMU 메시지에 설정
            corrected_yaw = yaw + yaw_offset
            imu_msg.orientation = self.euler_to_quaternion(0.0, 0.0, -corrected_yaw)

            # self.get_logger().info(f"Corrected yaw: {imu_msg.orientation}")

            # self.imu_publisher.publish(imu_msg)

        except (ValueError, IndexError) as e:
            self.get_logger().error(f"{parts}, Error: {e}")



    def calculate_position(self):
        A = 2 * (self.a2['x'] - self.a1['x'])
        B = 2 * (self.a2['y'] - self.a1['y'])
        C = (self.a1['d']**2 - self.a2['d']**2 - self.a1['x']**2 + self.a2['x']**2 - self.a1['y']**2 + self.a2['y']**2)
        D = 2 * (self.a3['x'] - self.a2['x'])
        E = 2 * (self.a3['y'] - self.a2['y'])
        F = (self.a2['d']**2 - self.a3['d']**2 - self.a2['x']**2 + self.a3['x']**2 - self.a2['y']**2 + self.a3['y']**2)
        X = ((F * B) - (E * C)) / ((B * D) - (E * A))
        Y = ((F * A) - (D * C)) / ((A * E) - (D * B))
        # self.get_logger().info(f" X = {X}, Y = {Y}")
        return X, Y
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy

        return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main(args=None):
    rclpy.init(args=args)
    test = Localization()
    rclpy.spin(test)
    test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()