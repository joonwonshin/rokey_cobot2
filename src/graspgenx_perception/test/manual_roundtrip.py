"""수동 통합 확인용. 컨테이너 안에서 yolo_seg_node 를 띄운 채로 실행한다.

    ros2 run graspgenx_perception yolo_seg_node --ros-args -p image_topic:=/yolo_seg_probe/image &
    python3 src/graspgenx_perception/test/manual_roundtrip.py <이미지경로>

이미지를 컬러 토픽으로 흘려보내고 /yolo_seg/labels 를 받아 라벨 분포를 찍는다.
pytest 대상이 아니다 (파일명이 test_ 로 시작하지 않는다) — 실제 GPU·토픽이 필요하다.

⚠️ 기본 발행 토픽은 `/yolo_seg_probe/image` 다. 실제 카메라 토픽
(`/camera/camera/color/image_raw`)에 쏘면 realsense2_camera·graspx 등 **다른 소비자에게도
합성 프레임이 간다.** 굳이 그러려면 두 번째 인자로 토픽을 명시할 것.
"""
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


DEFAULT_TOPIC = '/yolo_seg_probe/image'


class Probe(Node):
    def __init__(self, path, topic):
        super().__init__('yolo_seg_probe')
        self.bridge = CvBridge()
        self.img = cv2.imread(path)
        assert self.img is not None, f'이미지를 못 읽었다: {path}'
        self.pub = self.create_publisher(Image, topic, 10)
        self.create_subscription(Image, '/yolo_seg/labels', self.on_labels,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, '/yolo_seg/mask', self.on_mask, qos_profile_sensor_data)
        self.create_timer(0.5, self.tick)
        self.got = {}

    def tick(self):
        msg = self.bridge.cv2_to_imgmsg(self.img, encoding='bgr8')
        msg.header.frame_id = 'camera_color_optical_frame'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

    def on_labels(self, msg):
        a = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        self.got['labels'] = (a.shape, sorted(np.unique(a).tolist()))

    def on_mask(self, msg):
        a = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        self.got['mask'] = (a.shape, sorted(np.unique(a).tolist()))


def main():
    rclpy.init()
    topic = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_TOPIC
    node = Probe(sys.argv[1], topic)
    print('publish ->', topic)
    for _ in range(200):
        rclpy.spin_once(node, timeout_sec=0.1)
        if len(node.got) == 2:
            break
    print('입력 해상도:', node.img.shape[:2])
    for k, v in node.got.items():
        print(f'{k}: shape={v[0]} unique={v[1]}')
    if not node.got:
        print('아무것도 못 받았다 — 노드가 떠 있는지, DDS 설정이 맞는지 확인할 것')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
