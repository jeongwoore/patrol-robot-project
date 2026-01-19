import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from ultralytics import YOLO

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy

# 카메라 토픽은 BEST_EFFORT (지연 최소화)
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

class TrafficLightNode(Node):
    def __init__(self):
        super().__init__('traffic_light_node')

        # 신호등 학습 모델 (RED / GREEN)
        self.model = YOLO('/home/cho/lch_ws/src/turtle_pkg/turtle_pkg/patrol_robot/best.pt')
        self.current_state = "NO"              # RED / GREEN / NO
        self.time = self.get_clock().now()     # 프레임 주기 제어

        # 이미지 구독 / 상태 퍼블리셔
        self.sub_img = self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback, qos)
        self.pub_state = self.create_publisher(String, '/traffic_state', 10)

    def image_callback(self, msg):
        now = self.get_clock().now()

        if (now - self.time).nanoseconds < 300_000_000:
            return
        self.time = now

        # CompressedImage → OpenCV 이미지
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return

        results = self.model(frame, verbose=False)
        detected_status = "NO"

        for result in results:
            for box in result.boxes:
                cls = int(box.cls[0])
                name = self.model.names[cls].lower()
                conf = float(box.conf[0])

                # RED / GREEN 클래스만 처리
                if name not in ("red", "green"):
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                roi = frame[y1:y2, x1:x2]
                if roi.size == 0:
                    continue

                # HSV 색상 기반 필터링
                hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

                red1 = cv2.inRange(hsv, (0, 120, 70), (10, 255, 255))
                red2 = cv2.inRange(hsv, (170, 120, 70), (180, 255, 255))
                red_mask = red1 + red2

                green1 = cv2.inRange(hsv, (35, 50, 50), (85, 255, 255))
                green2 = cv2.inRange(hsv, (30, 30, 80), (90, 255, 255))
                green_mask = green1 + green2

                # 색상 비율 계산
                red_ratio = np.sum(red_mask > 0) / red_mask.size
                green_ratio = np.sum(green_mask > 0) / green_mask.size

                # 색상 기준
                if red_ratio > 0.15:
                    detected_status = "RED"
                    box_color = (0, 0, 255)
                elif green_ratio > 0.12:
                    detected_status = "GREEN"
                    box_color = (0, 255, 0)
                else:
                    continue

                # 시각화
                cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)
                cv2.putText(frame, f"{name} {conf:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)

        self.current_state = detected_status
        state_msg = String()
        state_msg.data = detected_status
        self.pub_state.publish(state_msg)

        # (표시용) 프레임 인코딩
        cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
