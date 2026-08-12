#!/usr/bin/env python3
"""
depth_downsample_node — aligned_depth_to_color 를 nvblox/robot_segmenter(T4/T5)용으로 다운샘플한다.

왜 필요한가
-----------
graspgenx(그래스핑 정밀도)는 depth 해상도가 높을수록 유리하고, robot_segmenter_node(T4,
컨테이너 GPU)는 픽셀당 연산이라 해상도가 낮을수록 반응이 빠르다(md/context 실측:
세그멘터가 9.65 Hz 입력을 3.7 Hz로 깎는 게 파이프라인 병목 — config/testcommand.md:257).
RealSense 드라이버는 depth를 **한 해상도로만** 낼 수 있어(camera.launch.py 주석 참고) 카메라
쪽에서 두 해상도를 동시에 뽑을 수 없다. 그래서 카메라는 graspgenx가 요구하는 고해상도로
한 번만 열고, 이 노드가 T4 직전에서 다운샘플해 저해상도 사본을 만든다.

방식
----
- depth: INTER_NEAREST만 쓴다. 선형보간은 물체 경계에서 전경/배경 깊이를 섞어
  존재하지 않는 중간 깊이 값을 만들어낸다(depth edge 아티팩트) — robot_segmenter의
  구체(sphere) 충돌 판정을 오염시킨다.
- camera_info: K(fx,fy,cx,cy)를 가로/세로 축소비로 스케일한다. D(왜곡계수)는 그대로
  둔다 — aligned_depth_to_color는 이미 rectify된 스트림이라 D는 원래도 0이다(RealSense 통상).
  width/height 필드도 다운샘플 후 값으로 고쳐 둔다(robot_segmenter는 K만 쓰지만,
  다른 소비자가 늘어날 걸 대비해 일관성 유지).

사용
----
    ros2 run m0609_rg2_bringup depth_downsample_node.py --ros-args \\
      -p target_width:=424 -p target_height:=240

기본 입력/출력 토픽은 camera.launch.py / testcommand.md T4~T5 예시와 맞춰뒀다.
출력 토픽을 T4의 depth_image_topics/depth_camera_infos 인자에 그대로 넣으면 된다.
"""
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class DepthDownsampleNode(Node):

    def __init__(self):
        super().__init__('depth_downsample_node')

        p = self.declare_parameter
        self.in_image_topic = p('in_image_topic',
                                 '/camera/camera/aligned_depth_to_color/image_raw').value
        self.in_info_topic = p('in_info_topic',
                                '/camera/camera/aligned_depth_to_color/camera_info').value
        self.out_image_topic = p('out_image_topic', '/cumotion/depth_1/image').value
        self.out_info_topic = p('out_info_topic', '/cumotion/depth_1/camera_info').value
        self.target_width = p('target_width', 424).value
        self.target_height = p('target_height', 240).value

        self._bridge = CvBridge()
        self._latest_info = None  # 원본 CameraInfo — depth 콜백마다 스케일해서 같이 낸다

        self._pub_image = self.create_publisher(
            Image, self.out_image_topic, qos_profile_sensor_data)
        self._pub_info = self.create_publisher(
            CameraInfo, self.out_info_topic, qos_profile_sensor_data)

        self.create_subscription(
            CameraInfo, self.in_info_topic, self._on_info, qos_profile_sensor_data)
        self.create_subscription(
            Image, self.in_image_topic, self._on_depth, qos_profile_sensor_data)

        self.get_logger().info(
            f'{self.in_image_topic} -> {self.out_image_topic} '
            f'({self.target_width}x{self.target_height}, INTER_NEAREST)')

    def _on_info(self, msg: CameraInfo):
        self._latest_info = msg

    def _on_depth(self, msg: Image):
        if self._latest_info is None:
            return  # 첫 camera_info가 올 때까지는 스케일할 K가 없다 — 프레임을 버린다

        depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        h0, w0 = depth.shape[:2]
        if w0 == self.target_width and h0 == self.target_height:
            resized = depth  # 이미 목표 해상도면 리사이즈를 건너뛴다
        else:
            resized = cv2.resize(
                depth, (self.target_width, self.target_height),
                interpolation=cv2.INTER_NEAREST)

        out_img = self._bridge.cv2_to_imgmsg(resized, encoding=msg.encoding)
        out_img.header = msg.header
        self._pub_image.publish(out_img)

        sx = self.target_width / w0
        sy = self.target_height / h0
        info = self._latest_info
        k = np.array(info.k, dtype=np.float64).reshape(3, 3)
        k[0, 0] *= sx  # fx
        k[1, 1] *= sy  # fy
        k[0, 2] *= sx  # cx
        k[1, 2] *= sy  # cy

        out_info = CameraInfo()
        out_info.header = msg.header
        out_info.width = self.target_width
        out_info.height = self.target_height
        out_info.distortion_model = info.distortion_model
        out_info.d = info.d
        out_info.k = k.flatten().tolist()
        # P도 같은 스케일로 맞춘다(fx,fy,cx,cy가 P[0,0]/P[1,1]/P[0,2]/P[1,2]와 같은 자리).
        p_mat = np.array(info.p, dtype=np.float64).reshape(3, 4)
        p_mat[0, 0] *= sx
        p_mat[1, 1] *= sy
        p_mat[0, 2] *= sx
        p_mat[1, 2] *= sy
        out_info.p = p_mat.flatten().tolist()
        out_info.r = info.r
        self._pub_info.publish(out_info)


def main():
    rclpy.init()
    node = DepthDownsampleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
