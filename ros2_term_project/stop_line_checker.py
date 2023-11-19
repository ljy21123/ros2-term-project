import cv2
import numpy


class StopLineChecker:
    def __init__(self):
        self._stopline = None

    def process(self, img: numpy.ndarray) -> None:
        """
        calculate the delta from the image
        :return: None
        """
        # 수정: HSV 색 공간으로 변경
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # 흰색의 HSV 범위
        lower_white = numpy.array([0, 0, 160])
        upper_white = numpy.array([255, 80, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)

        h, w, d = img.shape

        # 위에서 부터 높이 50
        search_top = int(70)
        # 위에서 부터 높이 50에서 +30한 위치
        search_bot = int(100)
        temp = mask.copy()
        # 0으로 처리
        mask[0:search_top, 0:w] = 0
        # 0으로 처리
        mask[search_bot:, 0:w] = 0

        self._stopline = False
        for y in range(h):
            white_pixels = cv2.countNonZero(mask[y, :])
            if white_pixels / w >= 0.7:
                self._stopline = True
                break

    @property
    def stopline(self) -> bool:
        return self._stopline


def main():
    tracker = StopLineChecker()
    import time
    for i in range(100):
        img = cv2.imread('sample.png')
        tracker.process(img)
        time.sleep(0.1)


if __name__ == "__main__":
    main()