#!/usr/bin/env python3
"""move_group이 계획한(=지금 execute()가 따라갈) 궤적을 tool0 기준 TCP 라인으로 그린다.

move_group은 `plan_only` 값과 무관하게(즉 `moveit_bridge.py`의 `move_to_joints_async`가
쓰는 plan+execute 단일 액션 호출에서도) 계획이 성공할 때마다
`/move_group/display_planned_path`(moveit_msgs/DisplayTrajectory)를 퍼블리시한다 — RViz
"Plan & Execute" 버튼을 눌러도 실행 전 애니메이션이 뜨는 게 이 토픽 덕분이다. 이 토픽만
구독하므로 **task_manager/moveit_bridge 쪽 실행 흐름은 전혀 건드리지 않는다.**

관절 궤적의 각 waypoint를 `/compute_fk`로 순방향기구학을 풀어 tool0 위치를 얻고, LINE_STRIP
마커로 이어 그린다. RViz의 MotionPlanning 패널(관절 애니메이션)은 그대로 유지한 채 위에
얹어지는 것이라, 관절 자세가 아니라 "TCP가 지나갈 직선 경로"만 명확히 보고 싶을 때 쓴다.

    ros2 run pick_fsm planned_tcp_path_node

발행:
    /pick/planned_tcp_path (visualization_msgs/Marker, LINE_STRIP)
        marker.id 고정 → 새 plan이 나올 때마다 RViz에서 이전 라인을 덮어쓴다(직접 지울 필요 없음).

⚠️ UNVERIFIED: "plan_only=False 조합 액션에서도 display_planned_path가 매번 나온다"는
move_group 소스 기반 가정이다. 실기에서 `ros2 topic hz /move_group/display_planned_path`로
실제 move_to_joints_async 호출 중 퍼블리시되는지 먼저 확인할 것.
"""

from moveit_msgs.msg import DisplayTrajectory, RobotState
from moveit_msgs.srv import GetPositionFK
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
import rclpy


class PlannedTcpPathNode(Node):
    def __init__(self):
        super().__init__('planned_tcp_path_node')

        self.declare_parameter('base_frame', 'base_link')  # moveit_bridge.py 의 base_frame과 통일
        self.declare_parameter('tip_link', 'tool0')         # SRDF end_effector parent_link
        self.declare_parameter('downsample', 1)             # 1=모든 점, N=N개마다 FK (마지막 점은 항상 포함)

        self.base_frame = self.get_parameter('base_frame').value
        self.tip_link = self.get_parameter('tip_link').value
        self.downsample = max(1, int(self.get_parameter('downsample').value))

        cbg = ReentrantCallbackGroup()
        self.fk = self.create_client(GetPositionFK, '/compute_fk', callback_group=cbg)
        self.sub = self.create_subscription(
            DisplayTrajectory, '/move_group/display_planned_path', self._on_plan, 10,
            callback_group=cbg)
        self.marker_pub = self.create_publisher(Marker, '/pick/planned_tcp_path', 10)

        self._generation = 0  # 겹쳐 들어오는 이전 plan의 FK 결과를 무시하기 위한 세대 번호
        self.get_logger().info(
            f'planned_tcp_path_node 시작: tip_link={self.tip_link}, base_frame={self.base_frame}')

    def _on_plan(self, msg: DisplayTrajectory):
        if not msg.trajectory or not msg.trajectory[0].joint_trajectory.points:
            return
        jt = msg.trajectory[0].joint_trajectory
        points = list(jt.points)[::self.downsample]
        if points[-1] is not jt.points[-1]:
            points.append(jt.points[-1])

        self._generation += 1
        gen = self._generation
        results = [None] * len(points)
        pending = [len(points)]  # 클로저에서 재할당하려고 리스트로 감쌈

        for i, pt in enumerate(points):
            req = GetPositionFK.Request()
            req.header.frame_id = self.base_frame
            req.fk_link_names = [self.tip_link]
            req.robot_state = RobotState()
            req.robot_state.joint_state = JointState()
            req.robot_state.joint_state.name = list(jt.joint_names)
            req.robot_state.joint_state.position = list(pt.positions)
            fut = self.fk.call_async(req)
            fut.add_done_callback(self._make_fk_done(gen, i, results, pending))

    def _make_fk_done(self, gen, i, results, pending):
        def cb(fut):
            if gen != self._generation:
                return  # 이미 새 plan이 들어와서 이 결과는 버린다
            resp = fut.result()
            if resp is not None and resp.error_code.val == resp.error_code.SUCCESS and resp.pose_stamped:
                results[i] = resp.pose_stamped[0].pose.position
            pending[0] -= 1
            if pending[0] == 0:
                self._publish_line(results)
        return cb

    def _publish_line(self, results):
        pts = [p for p in results if p is not None]
        if len(pts) < 2:
            self.get_logger().warn(f'FK 성공한 waypoint가 {len(pts)}개뿐 — 라인을 그리지 않음')
            return
        m = Marker()
        m.header.frame_id = self.base_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'planned_tcp_path'
        m.id = 0  # 고정 id → 새 plan마다 이전 라인을 덮어씀
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.008  # 선 굵기(m)
        m.color = ColorRGBA(r=0.1, g=0.9, b=0.2, a=0.9)
        m.points = pts
        m.lifetime.sec = 0  # 0 = 다음 덮어쓰기 전까지 계속 표시
        self.marker_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = PlannedTcpPathNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
