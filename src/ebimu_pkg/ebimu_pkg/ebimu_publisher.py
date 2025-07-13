import rclpy # ROS 2 Python 클라이언트 라이브러리 import
from rclpy.node import Node # ROS 2 노드를 만들기 위한 클래스
from visualization_msgs.msg import Marker # Rviz에서 시각화 할 수 있는 마커 메시지 
from sensor_msgs.msg import Imu # IMU 센서 데이터를 표현하는 메시지 타입 
from geometry_msgs.msg import Quaternion, Pose, Twist, TransformStamped # 로봇의 자세 및 위치 변환을 표현하는 메시지 타입
from nav_msgs.msg import Odometry # 로봇의 위치 및 속도를 표현하는 메시지 타입 
from tf2_ros import TransformBroadcaster # ROS 2에서 TF 변환을 PUBLISH하기 위한 브로드캐스터 클래스

import serial # 시리얼 포트를 통해 데이터를 읽기 위한 라이브러리
import math # 수학적 계산을 위한 라이브러리

# 'Node' 클래스를 상속하여  사용자 정의 노드
class EbimuPublisher(Node):
    def __init__(self): # 노드의 생성자 메서드 -> 노드의 초기화 작업 수행
        super().__init__('ebimu_publisher') # 노드 이름 설정 
        # 'imu/marker' 토픽으로 'Marker' 메시지를 퍼블리시할 PUBLISHER 생성 -> Rviz 시각화
        self.publisher = self.create_publisher(Marker, '/imu/marker', 10)
        # '/imu0' 토픽으로 'Imu' 메시지를 퍼블리시할 PUBLISHER 생성 -> IMU 데이터
        self.imu_publisher = self.create_publisher(Imu, '/imu', 10)
        # 0.01초 주기로 'timer_callback' 메서드 호출
        self.timer = self.create_timer(0.01, self.timer_callback)
        # '/dev/ttyUSB0' 포트에서 115200 baudrate로 시리얼 포트 설정
        self.serial_port = serial.Serial('/dev/ttyUSB0', baudrate=115200)
        # TF 변환을 PUBLISH 할 브로드캐스터 초기화
        # self.tf_broadcaster = TransformBroadcaster(self)  


    # 오일러 각을 쿼터니언으로 변환하는 메서드 #
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

        return Quaternion(x=qx, y=qy, z=qz, w=qw) # 계산된 쿼터니언 값 반환


    # 타이머 콜백 (0.01초 주기) #
    def timer_callback(self):
        roll=pitch=yaw=ax=ay=az=lx=ly=lz=0.0
        # IMU 센서와 포트 통신이 정상적으로 연결되어있는지 확인 
        if self.serial_port:
            # 직렬 포트로부터 한 줄의 데이터 출력
            line = self.serial_port.readline() 
            # 데이터를 'UTF-8'로  디코딩하여 문자열로 변환
            # 'errors=ignore'로 디코딩 중 오류가 발생해도 무시하고 계속 진행
            # 'strip()'를 통해 앞 뒤의 공백이나 개행 문자 제거
            line = line.decode('utf-8', errors='ignore').strip()
            # '*' 문자로 시작한다면, 해당 문자를 제거 -> 데이터의 시작을 나타내는 기호일 가능성이 있음
            if line.startswith('*'): 
                line = line[1:]
            try:
                # 데이터를 ','로 분리 -> (roll,pitch,yaw) 형식을 분리하여 데이터 추출
                parts = line.split(',')
                # 데이터가 3개로 나뉜 경우, 'float'으로 변환하여 변수에 할당 -> 오일러각 (회전 각도)
                if len(parts) == 9:
                  roll,pitch,yaw,ax,ay,az,lx,ly,lz = map(float, parts)
                # print(roll)
                # print(pitch)
                # print(yaw)
                print("-----------") # 구분을 위해 출력하는 줄

                # 오일러 각을 쿼터니언으로 변환 ('euler_to_quaternion' 함수 호출)
                quaternion = self.euler_to_quaternion(0.0, 0.0, yaw)


                imu_msg = Imu() # IMU 메시지 생성 -> IMU 데이터 발행
                imu_msg.header.frame_id = 'base_link' # 'base_link' 프레임 기준으로 IMU 데이터 설정
                imu_msg.header.stamp = self.get_clock().now().to_msg() # 현재 시간 설정

                # 가속도, 각속도 정보는 여기서 0으로 설정 -> 사용 X
                imu_msg.linear_acceleration.x = lx
                imu_msg.linear_acceleration.y = ly
                imu_msg.linear_acceleration.z = lz
                imu_msg.angular_velocity.x = ax
                imu_msg.angular_velocity.y = ay
                imu_msg.angular_velocity.z = az
                
                # 오일러 각에서 계산된 쿼터니언을 IMU 메시지에 설정 -> 로봇의 실제 자세
                imu_msg.orientation= quaternion
                # print(quaternion)
                
                # '/imu0' 토픽으로 IMU 메시지 퍼블리시
                self.imu_publisher.publish(imu_msg)

                # t = TransformStamped()
                # t.header.frame_id = "base_link"
                # t.child_frame_id = "imu_link"
                # t.header.stamp = self.get_clock().now().to_msg()
                
                # t.transform.translation.x = 0.0
                # t.transform.translation.y = 0.0
                # t.transform.translation.z = 0.0

                # t.transform.rotation = quaternion

                # self.tf_broadcaster.sendTransform(t)

                # print(quaternion)



            # 변환 과정에서 오류가 발생할 경우 
            except ValueError as e:
                self.get_logger().error(f"Failed to convert to float: {line}, Error: {str(e)}")



def main(args=None):
    rclpy.init(args=args)

    print("Starting ebimu_publisher..")

    node = EbimuPublisher()

    try:
        rclpy.spin(node)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()