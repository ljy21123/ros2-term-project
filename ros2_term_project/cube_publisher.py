#!/usr/bin/env python3
# ROS2용 패키지 임포트
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from geometry_msgs.msg import Twist
from threading import Thread, Event
import math


class CubePublisher(Node):  # Node 상속
    # 발행률 정적 변수 정의
    PUB_RATE = 10.0

    def __init__(self):
        # Node의 node_name 초기화
        super().__init__('cube_publisher')
        # Publisher 객체 생성
        self.publisher_ = self.create_publisher(Twist, '/CUBE/cmd_demo', 10)
        timer_period = 1 / CubePublisher.PUB_RATE  # seconds
        # Timer 객체 생성
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # 선속도 인스턴스 변수 정의
        self.current_linear_y = 1.0

        # Odometry 데이터를 받기 위한 subscriber 추가
        self.subscription = self.create_subscription(Odometry, '/CUBE/odom', self.odom_callback, 10)
        # 시작 위치 설정
        self.start_position = (34.997495, -64.0)
        # 목표 위치 설정
        self.target_position = (34.997495, -77.0)
        # 방향 전환 했는지 확인하는 변수
        self.turned_state = False

    # Timer 콜백 메서드 정의
    def timer_callback(self):
        msg = Twist()   # Twist 객체 생성
        msg.linear.y = float(self.current_linear_y) # 선속도 할당
        self.publisher_.publish(msg)    # 토픽 발행

    # Odometry 콜백 메서드 정의
    def odom_callback(self, msg):
        # 현재 위치 추출
        # pose.pose: 포즈 정보.실제 포즈 데이터
        current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        # 현재 위치와 목표 위치 간의 거리 계산
        distance_target = math.sqrt((self.target_position[0] - current_position[0]) ** 2 +
                                    (self.target_position[1] - current_position[1]) ** 2)
        distance_start = math.sqrt((self.start_position[0] - current_position[0]) ** 2 +
                                   (self.start_position[1] - current_position[1]) ** 2)

        # 일정 거리 내에 있으면 turn_cube 메서드 호출
        if distance_target < 0.01 and not self.turned_state:
            self.turn_cube()
            self.turned_state = True
        elif distance_start < 0.01 and self.turned_state:
            self.turn_cube()
            self.turned_state = False

    # 방향 전환 메서드
    def turn_cube(self):
        # while True:
        self.current_linear_y = -self.current_linear_y
        # self.get_logger().info('turn success!!')

    def stop(self):
        msg = Twist()  # Twist 객체 생성
        msg.linear.y = 0.0
        self.publisher_.publish(msg)  # 토픽 발행


def main(args=None):
    # 현재의 ROS2 컨텍스트에 대한 통신 초기화
    rclpy.init(args=args)
    # CubePublisher 객체 생성
    cube_publisher = CubePublisher()

    try:
        rclpy.spin(cube_publisher)
    except KeyboardInterrupt:
        cube_publisher.stop()

    # ROS2 컨텍스트 종료
    rclpy.shutdown()


main()


if __name__ == '__main__':
    main()