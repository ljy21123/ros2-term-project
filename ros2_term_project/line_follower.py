#!/usr/bin/env python3
import time


import rclpy
import sys
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from ros2_term_project.line_tracker import LineTracker
from ros2_term_project.stop_line_checker import StopLineChecker
from ros2_term_project.object_checker import ObjectChecker
from start_car.msg import StartCar
import cv_bridge
import threading


class LineFollower(Node):
    def __init__(self, line_tracker: LineTracker, stop_line_checker: StopLineChecker, object_checker: ObjectChecker, car_name):
        super().__init__('line_follower')
        self.car_name = car_name
        self.start_car_subscription = self.create_subscription(StartCar, '/start_car', self.start_car_callback, 10)
        self._subscription = None
        self._stop_line_checker_subscription = None
        self.lidar_subscription = None
        self._publisher = None
        self._time_publisher = None

        # 데이터 처리 객체
        self.stop_line_count = 0
        self.line_tracker = line_tracker
        self.stop_line_checker = stop_line_checker
        self.object_checker = object_checker
        self.bridge = cv_bridge.CvBridge()

        # 자동차 조작 변수
        self.twist = Twist()
        self.twist.linear.x = 5.0
        self.img = None
        # 장애물 회피 변수
        self.obstacle_found = False
        # 정지선 정지를 위한 변수
        self.speed = 5.0
        self.slowing_down = False
        self.parking = False
        self._stopline_coll = False
        self.hill = False

        self.t1 = None
        self.t2 = None

    def start_car_callback(self, msg:StartCar):
        if self.car_name == msg.car:
            # 라인주행 카메라
            self._subscription = self.create_subscription(Image, 'line_camera1/image_raw', self.image_callback, 10)
            # 정지선 카메라
            self._stop_line_checker_subscription = self.create_subscription(Image, 'stop_line_camera1/image_raw', self.stop_line_image_callback, 10)
            # 장애물 감지
            self.lidar_subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
            # 자동차 조작 토픽
            self._publisher = self.create_publisher(Twist, 'cmd_demo', 1)
            self._time_publisher = self.create_publisher(String, 'driving_time', 10)
            self.t1 = self.get_clock().now()

    def image_callback(self, msg: Image):
        # 장애물 발견시 이미지 처리 중지
        if self.obstacle_found: return
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        self.line_tracker.process(img)
        self.twist.angular.z = (-1) * self.line_tracker.delta / 410
        if self.twist.angular.z > 0.2:
            self.twist.linear.x = 3.0
        else:
            self.twist.linear.x = self.speed
        self._publisher.publish(self.twist)

    def scan_callback(self, msg: LaserScan):
        min_distance = min(msg.ranges)
        # 만약 7m 이내에 장애물 발견시
        if not self.obstacle_found and min_distance < 7.0:
            # self.get_logger().info("\n차량 앞에 무언가 있습니다.")
            self.object_checker.process(self.img)
            if self.object_checker.object:
                # self.get_logger().info("\n장애물 발견!")
                self.obstacle_found = True
                self.stop()
            else:
                pass
                # self.get_logger().info("\n장애물이 아닙니다.")
                # self.hill = True
        elif self.obstacle_found and min_distance > 7.0:
            self.obstacle_found = False

    def stop_line_image_callback(self, msg: Image):
        if not self._stopline_coll:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            timer = threading.Timer(4, self.setSpeed)
            timer2 = threading.Timer(1, self.rclpyShutdown)
            timer3 = threading.Timer(8, self.stopline_coll)
            self.img = img
            self.stop_line_checker.process(img)
            self.object_checker.process(img)
            if self.stop_line_checker.stopline:
                if self.stop_line_count == 2:
                    msg = String()
                    self.t2 = self.get_clock().now()
                    msg.data = f'{(self.t2 - self.t1).nanoseconds / 1e9:.3f}'
                    self._time_publisher.publish(msg)
                    self.stop()
                    timer2
                    timer2.start()
                if not self.slowing_down:
                    if self.hill:
                        self.speed = 0.2
                        self.slowing_down = True
                        timer.start()
                        self.hill = False
                    else:
                        self.speed = 0.0
                        self.slowing_down = True
                        timer.start()
                    self._stopline_coll = True
                    timer3.start()

    def stopline_coll(self):
        self._stopline_coll = False

    def setSpeed(self) -> None:
        self.stop_line_count += 1
        self.speed = 5.0
        timer = threading.Timer(3, self.setSlowingDown)
        timer.start()

    def setSlowingDown(self):
        self.slowing_down = False

    def rclpyShutdown(self):
        rclpy.shutdown()

    def stop(self):
        self.twist.linear.x = 0.0
        self._publisher.publish(self.twist)

    @property
    def publisher(self):
        return self._publisher


def main():
    rclpy.init()
    tracker = LineTracker()
    checker = StopLineChecker()
    object_checker = ObjectChecker()
    car_name = sys.argv[1]
    follower = LineFollower(tracker, checker, object_checker, car_name)
    try:
        rclpy.spin(follower)
    except KeyboardInterrupt:
        follower.stop()

if __name__ == '__main__':
    main()
