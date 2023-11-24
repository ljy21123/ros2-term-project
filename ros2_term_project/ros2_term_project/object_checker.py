import cv2
import numpy


class ObjectChecker:
    def __init__(self):
        self._object = False

    def process(self, img: numpy.ndarray) -> None:
        # 수정: HSV 색 공간으로 변경
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # 흰색의 HSV 범위
        lower_white = numpy.array([0, 0, 160])
        upper_white = numpy.array([255, 80, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        # 노란색 범위 설정
        lower_yellow = numpy.array([10, 100, 100])
        upper_yellow = numpy.array([50, 255, 250])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # 회색의 HSV 범위
        lower_gray = numpy.array([0, 0, 80])
        upper_gray = numpy.array([255, 40, 255])
        gray_mask = cv2.inRange(hsv, lower_gray, upper_gray)

        # 흰색과 회색을 합친 마스크
        combined_mask = cv2.bitwise_or(white_mask, gray_mask)
        combined_mask = cv2.bitwise_or(combined_mask, yellow_mask)

        # 반전된 마스크 (흰색과 회색을 제외한 영역)
        inverted_mask = cv2.bitwise_not(combined_mask)

        # 이미지에 반전된 마스크를 적용하여 흰색과 회색을 제거
        result = cv2.bitwise_and(img, img, mask=inverted_mask)

        has_object = cv2.countNonZero(inverted_mask) > 0

        # 객체가 탐지되면 self._object를 True로 설정
        self._object = has_object

    @property
    def object(self) -> bool:
        return self._object


def main():
    tracker = ObjectChecker()
    import time
    for i in range(100):
        img = cv2.imread('sample.png')
        tracker.process(img)
        time.sleep(0.1)


if __name__ == "__main__":
    main()