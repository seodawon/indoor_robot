import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import time

class ClockPublisher(Node):
    def __init__(self):
        super().__init__('clock_publisher')
        self.publisher_ = self.create_publisher(
            Clock, 
            '/clock', 
            QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.RELIABLE))
        
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        # 시스템 시간을 가져옴
        current_time = time.time()
        sec = int(current_time)
        nanosec = int((current_time - sec) * 1e9)

        # Clock 메시지 생성
        clock_msg = Clock()
        clock_msg.clock.sec = sec
        clock_msg.clock.nanosec = nanosec

        # Clock 메시지 발행
        self.publisher_.publish(clock_msg)
        # self.get_logger().info(f'Published clock: {sec}.{nanosec}')

def main(args=None):
    rclpy.init(args=args)
    clock_publisher = ClockPublisher()
    rclpy.spin(clock_publisher)
    clock_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
