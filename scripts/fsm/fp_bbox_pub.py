#!/usr/bin/env python3
"""고정 bbox를 컬러 프레임의 stamp를 복사해 Detection2D로 발행한다.

FoundationPose는 rgb/depth/camera_info/segmentation을 GXF sync로 묶고
기본 sync_threshold=0(완전 일치)이다(foundationpose_node.cpp:199).
따라서 stamp를 지어내면 한 프레임도 안 통과한다 — 실제 프레임의 header를 그대로 복사한다.

Detection2DToMask가 마스크의 header를 detection의 header에서 가져오므로
(detection2_d_to_mask.cpp:73) 여기서 복사한 stamp가 마스크까지 그대로 전달된다.

압축 토픽을 구독하는 이유: republish가 header를 그대로 복사하므로 stamp가 동일하고,
1.2MB raw를 다시 역직렬화하지 않아도 된다.

ponytail: RT-DETR 대신 고정 박스. 물체가 안 움직이는 bag 검증용이다.
          장면이 바뀌거나 다물체로 가면 isaac_ros_rtdetr로 교체한다(토픽 계약 동일).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import Detection2D

# vision_msgs 4.x: bbox.center 는 vision_msgs/Pose2D (position.x/y + theta) — 2026-08-05 실측 확인
DEFAULTS = {
    'image_topic': '/camera/camera/color/image_raw/compressed',
    'center_x': 424.0,   # 848x480 의 중앙
    'center_y': 240.0,
    'size_x': 240.0,
    'size_y': 240.0,
}


class FixedBboxPub(Node):
    def __init__(self):
        super().__init__('fp_fixed_bbox_pub')
        for k, v in DEFAULTS.items():
            self.declare_parameter(k, v)
        p = self.get_parameter
        self.pub = self.create_publisher(Detection2D, 'detection2_d', 10)
        self.sub = self.create_subscription(
            CompressedImage, p('image_topic').value, self.cb, 10)
        self.get_logger().info(
            f"bbox center=({p('center_x').value}, {p('center_y').value}) "
            f"size=({p('size_x').value}, {p('size_y').value}) "
            f"<- {p('image_topic').value}")

    def cb(self, msg):
        p = self.get_parameter
        det = Detection2D()
        det.header = msg.header          # stamp 복사 — 이 줄이 sync의 전부다
        det.bbox.center.position.x = float(p('center_x').value)
        det.bbox.center.position.y = float(p('center_y').value)
        det.bbox.size_x = float(p('size_x').value)
        det.bbox.size_y = float(p('size_y').value)
        self.pub.publish(det)


def main():
    rclpy.init()
    node = FixedBboxPub()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
