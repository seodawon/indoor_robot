import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import firebase_admin
from firebase_admin import db, credentials
import serial


class SimplePublisher(Node):

    def __init__(self):
        super().__init__('simple_publisher')

        # # Firebase 초기화
        # cred = credentials.Certificate("/home/sh/test_ws/ros2realtime-firebase-adminsdk-76yws-b2bbb1bb2f.json")
        # firebase_admin.initialize_app(cred, {
        #     'databaseURL': 'https://ros2realtime-default-rtdb.firebaseio.com'
        # })

        # # Firebase 데이터
        # self.ref = db.reference('admin/twist_data')

        # /cmd_vel 토픽 구독자 및 발행자 생성
        self.subscriber_ = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        # self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
    
        # 시리얼 통신 설정
        self.ser = serial.Serial('/dev/ttyUSB3', 115200)

        # 이전 Twist 데이터 저장 변수 초기화
        self.prev_twist_data = None

        # Firebase 데이터 변경이 되면 콜백 함수 불러옴
        # self.ref.listen(self.listener_callback)

        # 타이머 설정 (주기: 0.1초마다 실행)
        self.timer = self.create_timer(0.01, self.timer_callback)

        # 마지막 발행한 Twist 메시지 저장용 변수
        self.last_twist_msg = Twist()

    def cmd_vel_callback(self, msg):
        # teleop_keyboard에서 값을 주면 cmd_vel 토픽으로 가는데 그 값을 Firebase에 저장
        msg_dict = {
            "linear": {
                "x": msg.linear.x,
                "y": msg.linear.y,
                "z": msg.linear.z
            },
            "angular": {
                "x": msg.angular.x,
                "y": msg.angular.y,
                "z": msg.angular.z
            }
        }
        # Firebase 데이터베이스에 저장
        # self.ref.set(msg_dict)
        # self.last_twist_msg = msg  # 마지막으로 받은 메시지를 저장

    # def listener_callback(self, event):
    #     # Firebase 데이터 변경 시 호출되는 콜백 함수
    #     if event.data:
    #         twist_data = event.data
    #         # Twist 메시지 생성
    #         twist_msg = Twist()
    #         twist_msg.linear.x = float(twist_data['linear']['x'])
    #         twist_msg.linear.y = float(twist_data['linear']['y'])
    #         twist_msg.linear.z = float(twist_data['linear']['z'])
    #         twist_msg.angular.x = float(twist_data['angular']['x'])
    #         twist_msg.angular.y = float(twist_data['angular']['y'])
    #         twist_msg.angular.z = float(twist_data['angular']['z'])
    #         # 마지막으로 받은 메시지 업데이트
    #         self.last_twist_msg = twist_msg

    def timer_callback(self):
        # 주기적으로 cmd_vel 메시지 발행
        # self.publisher_.publish(self.last_twist_msg)
        
        # 시리얼로 모터 명령 전송
        wheel_separation = 0.602
        wl = ((self.last_twist_msg.linear.x * 3100) - (self.last_twist_msg.angular.z * 100))
        wr = ((self.last_twist_msg.linear.x * 3100) + (self.last_twist_msg.angular.z * 100))
        c = 0
        cmd = f"*{int(wl)},{int(wr)},{c},{0}\n"
        self.ser.write(cmd.encode())
        self.ser.flush()

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    try:
        rclpy.spin(simple_publisher)  # 노드 활성
    except KeyboardInterrupt:  # Ctrl+C 입력 시 예외 처리
        # 연결 종료 및 값 전송
        c = 1
        cmd = f"*{0},{0},{c},{0}\n"
        simple_publisher.ser.write(cmd.encode())
        simple_publisher.ser.close()  # 시리얼 포트 닫기
    simple_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
