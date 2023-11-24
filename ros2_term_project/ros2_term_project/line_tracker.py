import cv2
import numpy


class LineTracker:
    def __init__(self):
        self._delta = 0.0

    def process(self, img: numpy.ndarray) -> None:
        # 수정: HSV 색 공간으로 변경
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower_white = numpy.array([0, 0, 160])
        upper_white = numpy.array([255, 80, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)

        h, w, d = img.shape

        # 위에서 부터 높이의 75%
        search_top = int(3 * h / 4)
        # 위에서 부터 높이의 75%에서 +20한 위치 부터
        search_bot = int(3 * h / 4 + 14)
        # 0으로 처리
        mask[0:search_top, 0:w] = 0
        # 0으로 처리
        mask[search_bot:, 0:w] = 0
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(img, (cx, cy), 20, (0, 0, 255), -1)
            self.line = True  # 선이 있을 때 True로 설정
            # 제어 시작
            err = cx - w / 2
            self._delta = err
            # 제어 끝

    @property
    def delta(self):
        return self._delta

    @delta.setter
    def delta(self, delta):
        self._delta = delta


def main():
    tracker = LineTracker()
    import time
    for i in range(100):
        img = cv2.imread('sample2.png')
        tracker.process(img)
        if tracker.line:
            print("Line detected")
        else:
            print("No line detected")
        time.sleep(0.1)


if __name__ == "__main__":
    main()
