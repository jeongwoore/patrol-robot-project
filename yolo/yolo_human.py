import rclpy
import cv2
import numpy as np

from ultralytics import YOLO
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Int32
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node

qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

# 사람 객체 판단 노드
class PersonDetectNode(Node):
    def __init__(self):
        super().__init__('person_detect_node')

        # YOLOv8 사람 탐지 모델
        self.model = YOLO('/home/cho/lch_ws/src/turtle_pkg/turtle_pkg/patrol_robot/yolov8n.pt')
        
        self.pub_person = self.create_publisher(Int32, '/person', 10)
        self.pub_state = self.create_publisher(String, '/human_state', 10)

        # 압축 이미지 구독
        self.sub_img = self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback, qos)

        # 프레임 처리 주기 제어
        self.time = self.get_clock().now()
        self.detect_time = self.get_clock().now()

        self.detect_timeout = 0.6
        self.resize_width = 416

        self.person = None

    def image_callback(self, msg):
        now = self.get_clock().now()

        if (now - self.time).nanoseconds < 300_000_000:
            return
        self.time = now

        # CompressedImage → OpenCV 이미지
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return

        h, w, _ = frame.shape
        if w != self.resize_width:
            scale = self.resize_width / float(w)
            frame = cv2.resize(frame, (self.resize_width, int(h * scale)))

        frame_area = frame.shape[0] * frame.shape[1]

        detected = False
        max_area = 0.0

        # YOLO 추론
        results = self.model(frame, conf=0.8, verbose=False)

        for r in results:
            if r.boxes is None:
                continue

            for b in r.boxes:
                if int(b.cls[0]) != 0:
                    continue

                x1, y1, x2, y2 = b.xyxy[0].cpu().numpy().astype(int)
                area = max(0, x2 - x1) * max(0, y2 - y1)

                # 너무 작은 객체 제거
                if area < frame_area * 0.03:
                    continue

                # 가장 큰 사람 선택
                if area > max_area:
                    max_area = area
                    detected = True
                    self.person = (x1 + x2) // 2

        state_msg = String()
        human_msg = Int32()

        if detected:
            self.detect_time = now
            state_msg.data = "PERSON"
            human_msg.data = self.person
            self.pub_state.publish(state_msg)
            self.pub_person.publish(human_msg)

        else:
            if (now - self.detect_time).nanoseconds < int(self.detect_timeout * 1e9):
                return

            state_msg.data = "NORMAL"
            self.pub_state.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
