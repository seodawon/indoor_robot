import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from yolo_msgs.msg import  Local


class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.position = Local()
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        self.local_position = self.create_subscription(Local,'/local_position',self.local_callback,10)
        # Timer 설정 (1초마다 마커를 갱신)
        self.timer = self.create_timer(1.0, self.publish_marker)

    def local_callback(self,msg):
        self.position = msg
    
    def publish_marker(self):
        marker = Marker()
        
        # Header 설정
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        
        # 마커의 네임스페이스와 ID 설정 (여러 마커를 구분할 때 사용)
        marker.ns = "basic_shapes"
        marker.id = 0
        
        # 마커 타입 설정 (SPHERE)
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # 마커의 위치와 크기 설정
        marker.pose.position.x = self.position.robot_x
        marker.pose.position.y = self.position.robot_y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        
        # 마커 색상 설정
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # 투명도 (1.0은 불투명)
        
        # 생명주기 설정 (0.0은 영구적 표시)
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        # 마커 메시지 발행
        self.publisher_.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    marker_publisher = MarkerPublisher()
    rclpy.spin(marker_publisher)
    marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
