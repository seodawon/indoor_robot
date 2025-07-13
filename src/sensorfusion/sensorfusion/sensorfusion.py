import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from tf2_ros import TransformBroadcaster

import serial
import math

class SensorFusion(Node):
    def __init__(self):
        super().__init__('ebimu_publisher')
        self.publisher = self.create_publisher(Marker, '/marker', 10)
        self.imu_publisher = self.create_publisher(Imu, '/imu', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.serial_port = serial.Serial('/dev/ttyUSB1', baudrate=115200)

    def timer_callback(self):
        if self.serial_port:
            line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('*'):
                line = line[1:]
            try:
                parts = line.split(',')
                if len(parts) == 9:
                    roll, pitch, yaw, ax, ay, az, lx, ly, lz = map(float, parts)

                    # IMU 메시지 생성 및 설정
                    imu_msg = Imu()
                    imu_msg.header.frame_id = 'imu_link'
                    imu_msg.header.stamp = self.get_clock().now().to_msg()

                    # 가속도 및 각속도 설정
                    imu_msg.linear_acceleration.x = 0.0
                    imu_msg.linear_acceleration.y = 0.0
                    imu_msg.linear_acceleration.z = lz
                    imu_msg.angular_velocity.x = 0.0
                    imu_msg.angular_velocity.y = 0.0
                    imu_msg.angular_velocity.z = az

                    # yaw 보정 값 설정 (예: -90도 보정)
                    yaw_offset = -160  # 90도 왼쪽으로 회전

                    # 보정된 yaw 값을 쿼터니언으로 변환하여 IMU 메시지에 설정
                    corrected_yaw = yaw + yaw_offset
                    imu_msg.orientation = self.euler_to_quaternion(0.0, 0.0, -corrected_yaw)
                    
                    # 보정된 yaw 값을 로그로 출력하여 확인
                    self.get_logger().info(f"Corrected yaw: {imu_msg.orientation}")
                    
                    self.imu_publisher.publish(imu_msg)
                    
            except ValueError:
                self.get_logger().error("Invalid data format received from serial port.")

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
    print("Starting ebimu_publisher..")
    node = SensorFusion()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
