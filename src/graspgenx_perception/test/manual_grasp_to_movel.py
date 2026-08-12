#!/usr/bin/env python3
"""`/grasp/best_tcp` 를 받아 Doosan movel 서비스 콜 **명령줄을 출력만** 한다.

    source /opt/ros/humble/setup.bash && source install/setup.bash
    python3 src/graspgenx_perception/test/manual_grasp_to_movel.py

⚠️ 이 스크립트는 로봇을 움직이지 않는다. 붙여넣을 문자열을 만들 뿐이다.
   실행은 사람이 판단해서 한다 (~/.claude/CLAUDE.md §0 실기 안전).

왜 필요한가 — `/grasp/best*` 와 `move_line` 은 단위·표현이 셋 다 다르다:
    위치   m            -> mm            (x1000)
    자세   쿼터니언      -> ZYZ 오일러 deg (Doosan posx 규약)
    프레임 base_link(TF) -> DR_BASE(ref=0)

UNVERIFIED — 실기로 반드시 대조할 것 (아래 "검증" 참고):
  1. ROS `base_link` 원점/축 == DSR `DR_BASE` 인가
  2. 펜던트 TCP 가 RG2 손끝으로 설정돼 있는가. 안 돼 있으면 posx 는 **플랜지** 기준이고
     이 스크립트가 주는 좌표만큼 실제 손끝이 어긋난다
"""

import argparse
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


def quat_to_matrix(x, y, z, w):
    """capture_graspgenx_scene.quat_to_matrix 와 같은 식. 중복이지만 이 파일은
    ROS 패키지 import 없이도 돌아야 해서(수동 도구) 복사해 둔다."""
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])


def matrix_to_zyz_deg(R):
    """3x3 -> Doosan posx 의 (rx, ry, rz) [deg]. R = Rz(rx) Ry(ry) Rz(rz).

    유도: Rz(a)Ry(b)Rz(c) 를 전개하면
        R[0,2]=ca*sb, R[1,2]=sa*sb, R[2,2]=cb, R[2,0]=-sb*cc, R[2,1]=sb*sc
    """
    sy = math.hypot(R[0, 2], R[1, 2])
    if sy < 1e-9:                     # 짐벌: b≈0 또는 π. a 와 c 가 겹친다
        return (math.degrees(math.atan2(R[1, 0], R[0, 0])),
                0.0 if R[2, 2] > 0 else 180.0, 0.0)
    return (math.degrees(math.atan2(R[1, 2], R[0, 2])),
            math.degrees(math.atan2(sy, R[2, 2])),
            math.degrees(math.atan2(R[2, 1], -R[2, 0])))


def to_posx(pose):
    """geometry_msgs/Pose -> [x,y,z,rx,ry,rz] (mm, deg)."""
    o = pose.orientation
    rx, ry, rz = matrix_to_zyz_deg(quat_to_matrix(o.x, o.y, o.z, o.w))
    return [pose.position.x * 1000.0, pose.position.y * 1000.0, pose.position.z * 1000.0,
            rx, ry, rz]


def command(posx, ns, vel, acc):
    pos = ', '.join(f'{v:.2f}' for v in posx)
    return (f"ros2 service call {ns}/motion/move_line dsr_msgs2/srv/MoveLine "
            f"\"{{pos: [{pos}], vel: [{vel}, {vel}], acc: [{acc}, {acc}], "
            f"time: 0.0, radius: 0.0, ref: 0, mode: 0, blend_type: 0, sync_type: 0}}\"")


class Printer(Node):
    def __init__(self, args):
        super().__init__('grasp_to_movel')
        self.args = args
        self.create_subscription(PoseStamped, args.topic, self.on_pose, 10)
        self.get_logger().info(f'{args.topic} 대기 중… (ros2 service call /grasp/compute ...)')

    def on_pose(self, msg):
        posx = to_posx(msg.pose)
        approach = list(posx)
        approach[2] += self.args.approach_mm      # 접근점은 그냥 z 를 띄운다(base 기준)
        print(f"\nframe_id={msg.header.frame_id}  (ref=0 DR_BASE 와 같아야 한다)")
        print(f"posx = [{', '.join(f'{v:.2f}' for v in posx)}]")
        print(f"\n# 1) 접근점 (+{self.args.approach_mm:.0f} mm)\n"
              f"{command(approach, self.args.ns, self.args.vel, self.args.acc)}")
        print(f"\n# 2) 파지점 — ⚠️ 그리퍼 열고, 충돌 확인하고, 사람이 승인한 뒤에만\n"
              f"{command(posx, self.args.ns, self.args.vel, self.args.acc)}\n")


def main():
    ap = argparse.ArgumentParser()
    # best_tcp = 손끝. best = 그리퍼 base 라 접근이 기울면 물체에서 크게 벗어난다
    ap.add_argument('--topic', default='/grasp/best_tcp')
    ap.add_argument('--ns', default='/dsr01')
    ap.add_argument('--vel', type=float, default=50.0)     # 느리게. 실기 첫 시도용
    ap.add_argument('--acc', type=float, default=50.0)
    ap.add_argument('--approach_mm', type=float, default=100.0)
    args = ap.parse_args()

    rclpy.init()
    node = Printer(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
