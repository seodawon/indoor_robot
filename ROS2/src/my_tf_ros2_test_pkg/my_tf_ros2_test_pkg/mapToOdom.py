import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, Twist
from tf2_ros import TransformBroadcaster
import math
import serial
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from yolo_msgs.msg import Local
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from std_msgs.msg import Float32,Bool

class MapToOdomBroadcaster(Node):
    def __init__(self):
        super().__init__('map_to_odom')

        # TF 변환을 위한 Broadcaster 초기화
        self.map_to_odom_broadcaster = TransformBroadcaster(self)

        # /imu, /localization을 처리하기 위한 callback group 선언
        self.clock_group = ReentrantCallbackGroup()
        self.sensor_group = ReentrantCallbackGroup()

        try: # 시리얼 포트 설정
            self.ser = serial.Serial('/dev/ttyUSB1', 115200)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.ser = None  # 시리얼 포트 없이 계속 진행
        
        # 센서들에 대한 subscription 선언
        self.localization = self.create_subscription(
            Local,
            '/local_position',
            self.localization_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group = self.sensor_group
            )
        
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group = self.sensor_group
        )

        self.clock_subscriber = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.RELIABLE),
            callback_group = self.clock_group
        )

        # /initialpose 토픽을 구독하여 초기 위치 설정
        self.initial_pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initial_pose_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.RELIABLE),
            callback_group=self.sensor_group
        )
        
        self.robot = self.create_subscription(
            Bool,
            '/robot',
            self.robot_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.RELIABLE),
            callback_group=self.sensor_group
        )
        
        # cmd_vel 구독하여 속도 데이터 수신
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group = self.sensor_group
        )

        self.odom_publisher = self.create_publisher(
            Odometry,
            '/odom',
            QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group = self.sensor_group
        )
        self.robot_state= Bool()
        self.robot_state.data = True
        # 초기 위치 설정 
        self.position = Local()
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_z = 0.0
        self.yaw = 0.0
        
        # localization 값 초기화
        self.localization_x = 0.0
        self.localization_y = 0.0
        self.localization_z = 0.0

        # /cmd_vel 속도 정보
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # dt값 계산을 위한 시간값 초기화
        self.last_timestamp = self.get_clock().now()  # 초기 last_timestamp 설정

        # 센서 데이터를 담을 변수 초기화
        self.imu_data = Imu()
        self.clock_data = Clock()
        self.initial_pose= PoseWithCovarianceStamped()

        # TF 회전 값 초기화
        self.quaternion = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

    def robot_callback(self,msg):
        self.robot_state = msg


    # localization callback 함수
    def localization_callback(self, msg):
        self.position = msg

        if self.position.robot_x != 0.0:
           self.localization_x = self.position.robot_x
           self.localization_y = self.position.robot_y
           self.localization_z = 0.0


    def cmd_vel_callback(self, cmd_vel_msg):
        # cmd_vel이 들어오면 현재 속도 정보 갱신
        self.linear_velocity = cmd_vel_msg.linear.x
        self.angular_velocity = cmd_vel_msg.angular.z

        # IMU에서 yaw 값 계산 (orientation -> euler 변환)
        if self.imu_data.orientation is not None:
            q = self.imu_data.orientation
            self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        else:
            self.yaw = 0.0


    def imu_callback(self, imu_msg):
        self.imu_data = imu_msg


    def clock_callback(self, clock_msg):
        self.clock_data = clock_msg
        
        current_timestamp = self.get_clock().now()  # 현재 타임스탬프
        dt = (current_timestamp - self.last_timestamp).nanoseconds / 1e9  # 초 단위로 시간 차 계산
        self.last_timestamp = current_timestamp  # 마지막 타임스탬프 업데이트    

        self.send_motor_commands()  # 속도 변경 시 모터 명령 전송
        self.mapToOdom(dt)


    def initial_pose_callback(self, initial_pose_msg):
        self.initial_pose = initial_pose_msg

        self.pose_x = self.initial_pose.pose.pose.position.x
        self.pose_y = self.initial_pose.pose.pose.position.y
        self.pose_z = self.initial_pose.pose.pose.position.z

        self.get_logger().info(f'Initial pose set to x: {self.pose_x}, y: {self.pose_y}, z: {self.pose_z}')   


    # TF 변환 함수 (map -> odom)
    def mapToOdom(self, dt):
        
        if self.imu_data is None:
           self.get_logger().warn('IMU data is not available yet.')
           return
        
        if self.position is None:
           self.get_logger().warn('Localization data is not available yet.')
           return
        
        delta_x = self.linear_velocity * dt * math.cos(self.yaw)
        delta_y = self.linear_velocity * dt * math.sin(self.yaw)

        # 위치 갱신
        self.pose_x += delta_x
        self.pose_y += delta_y
        
        tf_mapToOdom = TransformStamped()

        clock_timestamp = self.get_clock().now().to_msg()

        # '/clock' 토픽의 timestamp를 사용하여 '/clock' 시간과 동기화 
        tf_mapToOdom.header.stamp = clock_timestamp
        tf_mapToOdom.header.frame_id = 'map'
        tf_mapToOdom.child_frame_id = 'odom'


        if self.position.robot_x != 0.0:
           self.pose_x = self.localization_x
           self.pose_y = self.localization_y
           self.pose_z = 0.0

        tf_mapToOdom.transform.translation.x = self.pose_x
        tf_mapToOdom.transform.translation.y = self.pose_y
        tf_mapToOdom.transform.translation.z = self.pose_z

        tf_mapToOdom.transform.rotation = self.imu_data.orientation

        self.map_to_odom_broadcaster.sendTransform(tf_mapToOdom)


        # '/odom' 토픽
        odom = Odometry()
        odom.header.stamp = clock_timestamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.pose_x
        odom.pose.pose.position.y = self.pose_y
        odom.pose.pose.position.z = self.pose_z

        odom.pose.pose.orientation = self.imu_data.orientation


        # Odometry 메시지 발행
        self.odom_publisher.publish(odom)


    def send_motor_commands(self):
        if self.ser is None:
            self.get_logger().warn("Serial port not available.")
            return
        
        if self.linear_velocity == 0.0 and self.angular_velocity == 0.0:
            wl = 0
            wr = 0
        else:
            wheel_separation = 0.602
            wl = (self.linear_velocity * 5100)
            wr = (self.linear_velocity * 5100)

        if self.robot_state.data == True:
            f=1
        else:
            f=0
        c = 1
        cmd = f"*{int(wl)},{int(wr)},{c},{f}\n"
        
        self.ser.write(cmd.encode())
        self.ser.flush()


def main(args=None):
    rclpy.init(args=args)
    tf_mapToOdom = MapToOdomBroadcaster()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(tf_mapToOdom)
    try:
        executor.spin()
    finally:
        if tf_mapToOdom.ser:
            c = 0
            cmd = f"*{0},{0},{c},{0}\n"
            tf_mapToOdom.ser.write(cmd.encode())
            tf_mapToOdom.ser.close()
        executor.shutdown()
        tf_mapToOdom.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()       