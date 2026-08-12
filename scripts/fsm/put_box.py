#!/usr/bin/env python3
"""planning scene에 상자 장애물을 넣는다 / 뺀다 — RViz Scene Objects의 CLI판.

RViz GUI 없이 "장애물이 실제로 회피 알고리즘에 작용하는가"를 확인하는 수단이다.
GUI로 놓은 상자는 눈에 보여도 계획에 안 먹는 경우가 있어(publish_geometry_updates 등),
스크립트로 넣고 /monitored_planning_scene에서 되읽는 쪽이 판정이 확실하다.

    ros2 launch m0609_rg2_moveit moveit.launch.py       # move_group이 떠 있어야 한다
    python3 scripts/put_box.py                          # 로봇 앞을 막는 벽
    python3 scripts/put_box.py --x 0.4 --size 0.1 0.8 0.5
    python3 scripts/put_box.py --remove

확인:
    ros2 topic echo /monitored_planning_scene --once | grep -A5 debug_box
    → 안 나오면 안 들어간 것이다. FRAME을 먼저 의심할 것.

⚠️ FRAME은 반드시 'base_link'다. 'world'로 주면 move_group이
   "[ERROR] Unknown frame: world"를 뱉고 상자를 **조용히 무시**한다.
   SRDF에 virtual_joint(parent_frame="world")가 있어도 그렇다 — MoveIt은 fixed 타입
   virtual joint로 모델 프레임을 만들지 않아 플래닝 프레임이 루트 링크로 남는다.
   (2026-08-02 실측. md/context/constraints.md "플래닝 프레임" 참고)
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive

FRAME = 'base_link'          # 위 경고 참고 — 'world'로 바꾸지 말 것
OBJECT_ID = 'debug_box'


def build_scene(args):
    co = CollisionObject()
    co.header.frame_id = FRAME
    co.id = OBJECT_ID

    if args.remove:
        co.operation = CollisionObject.REMOVE
    else:
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(args.size)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = args.x, args.y, args.z
        pose.orientation.w = 1.0
        co.primitives = [box]
        co.primitive_poses = [pose]
        co.operation = CollisionObject.ADD

    scene = PlanningScene()
    scene.world.collision_objects = [co]
    scene.is_diff = True        # 기존 장면을 덮지 않고 이 상자만 얹는다
    return scene


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--x', type=float, default=0.5, help=f'{FRAME} 기준 x [m]')
    p.add_argument('--y', type=float, default=0.0, help='y [m]')
    p.add_argument('--z', type=float, default=0.4, help='z [m]')
    p.add_argument('--size', type=float, nargs=3, default=[0.2, 0.6, 0.6],
                   metavar=('X', 'Y', 'Z'), help='상자 크기 [m]')
    p.add_argument('--remove', action='store_true', help='넣지 않고 뺀다')
    args = p.parse_args()

    rclpy.init()
    node = Node('put_box')
    pub = node.create_publisher(PlanningScene, '/planning_scene', 10)

    # move_group이 구독을 붙일 때까지 기다린다. 안 기다리면 첫 발행이 그냥 버려져서
    # "명령은 성공했는데 상자가 없다"가 된다 (latch가 아니다).
    deadline = time.time() + 5.0
    while pub.get_subscription_count() == 0 and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        raise SystemExit('/planning_scene 구독자 없음 — move_group이 떠 있는지 확인할 것')

    scene = build_scene(args)
    for _ in range(3):
        pub.publish(scene)
        rclpy.spin_once(node, timeout_sec=0.1)

    verb = '제거' if args.remove else '추가'
    print(f"{OBJECT_ID} {verb} 발행 완료 (frame={FRAME})")
    print("확인: ros2 topic echo /monitored_planning_scene --once | grep -A5 debug_box")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
