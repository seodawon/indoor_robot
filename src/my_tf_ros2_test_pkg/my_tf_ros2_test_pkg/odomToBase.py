import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster
import math
import serial
from geometry_msgs.msg import Quaternion
from rosgraph_msgs.msg import Clock
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

class OdomToBaseBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_to_base')

        # TF 변환을 위한 Broadcaster 초기화
        self.odom_to_base_broadcaster = TransformBroadcaster(self)
        
        self.clock_group = ReentrantCallbackGroup()
        self.cmd_vel_group = ReentrantCallbackGroup()
        
        # 시리얼 포트 설정
        try:
            self.ser = serial.Serial('/dev/ttyUSB1', 115200)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.ser = None  # 시리얼 포트 없이 계속 진행

        # cmd_vel 구독하여 속도 데이터 수신
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group = self.cmd_vel_group
        )

        self.clock_subscriber = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.RELIABLE),
            callback_group = self.clock_group
        )

        # 초기 위치 설정
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # 속도 정보
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.last_timestamp = self.get_clock().now()  # 초기 last_timestamp 설정

    def cmd_vel_callback(self, cmd_vel_msg):
        self.linear_velocity = cmd_vel_msg.linear.x
        self.angular_velocity = cmd_vel_msg.angular.z

        self.send_motor_commands()  # 속도 변경 시 모터 명령 전송

    def clock_callback(self, clock_msg):
        self.clock_data = clock_msg
        
        current_timestamp = self.get_clock().now()  # 현재 타임스탬프
        dt = (current_timestamp - self.last_timestamp).nanoseconds / 1e9  # 초 단위로 시간 차 계산
        self.last_timestamp = current_timestamp  # 마지막 타임스탬프 업데이트

        self.odomToBase(dt)

    # TF 변환 함수 (odom -> base_link)
    def odomToBase(self, dt):

        delta_x = self.linear_velocity * dt * math.cos(self.yaw)
        delta_y = self.linear_velocity * dt * math.sin(self.yaw)

        # 위치 갱신
        self.x += delta_x
        self.y += delta_y

        clock_timestamp = self.get_clock().now().to_msg()

        tf_odomToBase = TransformStamped()

        # '/clock' 토픽의 timestamp를 사용하여 '/clock' 시간과 동기화 
        tf_odomToBase.header.stamp = clock_timestamp
        tf_odomToBase.header.frame_id = 'odom'
        tf_odomToBase.child_frame_id = 'base_link'

        tf_odomToBase.transform.translation.x = self.x
        tf_odomToBase.transform.translation.y = self.y
        tf_odomToBase.transform.translation.z = 0.0

        self.odom_to_base_broadcaster.sendTransform(tf_odomToBase)

    def euler_to_quaternion(self, roll, pitch, yaw):
        # Roll, Pitch, Yaw를 Quaternion으로 변환
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def send_motor_commands(self):
        if self.ser is None:
            self.get_logger().warn("Serial port not available.")
            return
    
        if self.linear_velocity == 0.0 and self.angular_velocity == 0.0:
            wl = 0
            wr = 0
        else:
            wheel_separation = 0.602
            # wl = (self.linear_velocity * 3100) - (self.angular_velocity * 100)
            # wr = (self.linear_velocity * 3100) + (self.angular_velocity * 100)
            wl = (self.linear_velocity * 9300)
            wr = (self.linear_velocity * 9300)
        c = 1
        cmd = f"*{int(wl)},{int(wr)},{c},{0}\n"
        self.ser.write(cmd.encode())
        self.ser.flush()


def main(args=None):
    rclpy.init(args=args)
    tf_odomToBase = OdomToBaseBroadcaster()
    # callback 간의 충돌과 프로세스 지연을 방지하기 위한 callback group 구분과 멀티 스레드
    # excutor을 선언을 안 해주면 기본적으로 SingleThreadExecutor로 설정되지만, 여기서는 명시적으로 선언
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(tf_odomToBase) # 생성된 노드를 executor에 넣음
    try:
    # Spin the executor
        executor.spin() # 노드가 아니라 executor를 선언했기 때문에 executor.spin()을 선언
    finally:
        if tf_odomToBase.ser:
            c = 0
            cmd = f"*{0},{0},{c},{0}\n"
            tf_odomToBase.ser.write(cmd.encode())
            tf_odomToBase.ser.close()
        executor.shutdown()
        tf_odomToBase.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()