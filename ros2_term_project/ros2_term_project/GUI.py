import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from PySide2.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel, QMessageBox
from start_car.msg import StartCar

class Gui(QWidget):
    def __init__(self):
        super().__init__()
        # 0: 주행 전, 1: 주행 중, 2: 주행 완료
        self.b1 = 0
        self.b2 = 0
        self.initUI()

    def initUI(self):
        # 수직 레이아웃 생성
        layout = QVBoxLayout()

        # 첫 번째 라벨과 대기중 버튼 생성
        self.label1 = QLabel('PR001 - 대기중', self)
        self.label1.setStyleSheet("font-weight: bold;")
        layout.addWidget(self.label1)

        self.result_label1 = QLabel('주행 결과: 주행 전 입니다.', self)  # 주행 결과를 표시할 라벨 추가
        layout.addWidget(self.result_label1)

        self.button1 = QPushButton('PR001 주행 시작', self)
        self.button1.clicked.connect(self.button1Clicked)
        layout.addWidget(self.button1)

        # 두 번째 라벨과 대기중 버튼 생성
        self.label2 = QLabel('PR002 - 대기중', self)
        self.label2.setStyleSheet("font-weight: bold;")
        layout.addWidget(self.label2)

        self.result_label2 = QLabel('주행 결과: 주행 전 입니다.')  # 주행 결과를 표시할 라벨 추가
        layout.addWidget(self.result_label2)

        self.button2 = QPushButton('PR002 주행 시작', self)
        self.button2.clicked.connect(self.button2Clicked)
        layout.addWidget(self.button2)

        # 레이아웃 설정
        self.setLayout(layout)

        # 윈도우 설정
        self.setGeometry(300, 300, 300, 200)
        self.setWindowTitle('시뮬레이터 제어창')
        self.show()

    def set_result_label1(self, msg):
        self.result_label1.setText('주행 결과: ' + str(msg.data) + '초')
        self.label1.setText('PR001 - 주행 완료 ')
        self.b1 = 2

    def set_result_label2(self, msg):
        self.result_label2.setText('주행 결과: ' + str(msg.data) + '초')
        self.label2.setText('PR002 - 주행 완료 ')
        self.b2 = 2

    def run(self, target):
        temp = StartCarPub(target)

    def button1Clicked(self):
        if self.b1 == 0:
            thread = threading.Thread(target=self.run, args=('PR001',))
            thread.start()
            self.label1.setText('PR001 - 주행중')
            self.result_label1.setText('주행 결과: 결과 대기 중...')
            self.b1 = 1
        elif self.b1 == 1:
            self.showPopup('PR001 주행 중 입니다.')
        else:
            self.showPopup('PR001 주행 완료 상태입니다.')

    def button2Clicked(self):
        if self.b2 == 0:
            thread = threading.Thread(target=self.run, args=('PR002',))
            thread.start()
            self.label2.setText('PR002 - 주행중')
            self.result_label2.setText('주행 결과: 결과 대기 중...')
            self.b2 = 1
        elif self.b2 == 1:
            self.showPopup('PR002 주행 중 입니다.')
        else:
            self.showPopup('PR002 주행 완료 상태입니다.')

    def showPopup(self, message):
        msg = QMessageBox()
        msg.setWindowTitle('알림')
        msg.setText(message)
        msg.exec_()

    def exitClicked(self):
        sys.exit()


class StartCarPub(Node):
    def __init__(self, car):
        # Node의 node_name 초기화
        super().__init__('start', namespace=car)
        # Publisher 객체 생성
        self.publisher = self.create_publisher(StartCar, '/start_car', 10)
        self.msg = StartCar()
        self.msg.car = car
        # 컴퓨터 사양에 따라서 토픽을 제대로 전달 받지 못하는 경우가 생기기 때문에 여러번 발행
        for _ in range(5):
            self.publisher.publish(self.msg)
            time.sleep(0.1)
        # self.publisher.publish(self.msg)
        self.destroy_node()


class GuiRun:
    def __init__(self):
        self.ex = None
        self.lock = threading.Lock()
        self.lock2 = threading.Lock()

    def run(self):
        thread = threading.Thread(target=self.thread_run)
        thread.start()

    def thread_run(self):
        app = QApplication(sys.argv)
        self.ex = Gui()
        sys.exit(app.exec_())

    def update_gui_text(self, new_text):
        with self.lock:
            if self.ex:
                self.ex.set_result_label1(new_text)

    def update_gui_text2(self, new_text):
        with self.lock2:
            if self.ex:
                self.ex.set_result_label2(new_text)

def main():
    app = QApplication(sys.argv)
    gui = Gui()
    sys.exit(app.exec_())