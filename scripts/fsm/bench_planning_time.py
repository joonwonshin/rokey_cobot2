#!/usr/bin/env python3
"""OMPL vs cuMotion 계획 시간 측정 (게이트 E §7).

move_group의 MoveGroup 액션을 직접 부른다. RViz 드롭다운으로 사람이 번갈아 누르는 대신
`pipeline_id`만 바꿔 같은 목표를 N회 계획한다 — 사람 손 타이밍이 섞이지 않는다.

⚠️ **plan_only=True 고정이다. 로봇은 움직이지 않는다.** 실행은 이 스크립트가 하지 않는다.

컨테이너 안에서:
    source /opt/ros/humble/setup.bash
    source /workspaces/isaac_ros-dev/install/setup.bash
    source /workspaces/cobot2_ws/install_container/setup.bash
    export ROS_DOMAIN_ID=93
    python3 /workspaces/cobot2_ws/scripts/bench_planning_time.py --repeat 10

두 숫자가 나온다:
  wall   — 요청부터 결과 수신까지(액션 왕복·직렬화 포함). 사용자가 체감하는 시간
  server — MoveGroup 결과의 planning_time. 플래너 자신이 잰 시간
cuMotion은 첫 호출에 커널 워밍업이 섞일 수 있어 1회차를 따로 표시한다.
"""

import argparse
import statistics
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, PlanningOptions

# M0609 6축. 시작 자세는 move_group이 /joint_states에서 읽는 "현재" 자세다.
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
GOAL_RAD = [0.5, -0.4, 1.2, 0.0, 0.9, 0.0]


def build_goal(pipeline_id: str, group: str, allowed_time: float) -> MoveGroup.Goal:
    constraints = Constraints()
    for name, value in zip(JOINT_NAMES, GOAL_RAD):
        constraints.joint_constraints.append(JointConstraint(
            joint_name=name,
            position=value,
            tolerance_above=0.001,
            tolerance_below=0.001,
            weight=1.0,
        ))

    request = MotionPlanRequest()
    request.group_name = group
    request.pipeline_id = pipeline_id      # ← 이 한 줄이 OMPL/cuMotion을 가른다
    request.num_planning_attempts = 1
    request.allowed_planning_time = allowed_time
    request.max_velocity_scaling_factor = 0.1
    request.max_acceleration_scaling_factor = 0.1
    request.goal_constraints.append(constraints)

    options = PlanningOptions()
    options.plan_only = True               # ⚠️ 실행 금지. 절대 True 이외로 바꾸지 말 것
    options.planning_scene_diff.is_diff = True
    options.planning_scene_diff.robot_state.is_diff = True

    goal = MoveGroup.Goal()
    goal.request = request
    goal.planning_options = options
    return goal


def plan_once(node: Node, client: ActionClient, goal: MoveGroup.Goal, timeout: float):
    """(wall_ms, server_ms, error_code) 또는 실패 시 (wall_ms, None, code)."""
    started = time.perf_counter()
    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=timeout)
    handle = send_future.result()
    if handle is None or not handle.accepted:
        return (time.perf_counter() - started) * 1e3, None, 'REJECTED'

    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=timeout)
    wall_ms = (time.perf_counter() - started) * 1e3
    outcome = result_future.result()
    if outcome is None:
        return wall_ms, None, 'TIMEOUT'

    result = outcome.result
    code = result.error_code.val
    if code != 1:                          # 1 = SUCCESS
        return wall_ms, None, f'ERROR({code})'
    return wall_ms, result.planning_time * 1e3, 'OK'


def report(label, samples):
    if not samples:
        print(f'{label:22s}  측정값 없음')
        return
    first = samples[0]
    rest = samples[1:] or samples
    print(f'{label:22s}  1회차 {first:8.1f} ms   '
          f'이후 중앙값 {statistics.median(rest):8.1f} ms   '
          f'min {min(rest):8.1f}  max {max(rest):8.1f}  (n={len(samples)})')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--repeat', type=int, default=10)
    parser.add_argument('--group', default='manipulator')
    parser.add_argument('--allowed-time', type=float, default=5.0)
    parser.add_argument('--timeout', type=float, default=60.0,
                        help='액션 응답 대기 상한(초). cuMotion 첫 호출은 워밍업으로 느릴 수 있다')
    parser.add_argument('--pipelines', nargs='+', default=['ompl', 'isaac_ros_cumotion'])
    parser.add_argument('--action', default='/move_action',
                        help='move_group의 MoveGroup 액션 이름')
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('plan_time_bench')
    # 액션 이름은 '/move_group'이 아니라 '/move_action'이다 (move_group 노드가 여는 이름).
    client = ActionClient(node, MoveGroup, args.action)
    if not client.wait_for_server(timeout_sec=15.0):
        node.get_logger().error(f'{args.action} 액션 서버가 없다. move_group이 떠 있는지, '
                                'ROS_DOMAIN_ID/RMW_IMPLEMENTATION이 같은지 확인할 것.')
        rclpy.shutdown()
        return 1

    results = {}
    for pipeline in args.pipelines:
        walls, servers, failures = [], [], []
        for i in range(args.repeat):
            wall_ms, server_ms, status = plan_once(
                node, client, build_goal(pipeline, args.group, args.allowed_time), args.timeout)
            if status != 'OK':
                failures.append(f'#{i + 1} {status}')
                continue
            walls.append(wall_ms)
            servers.append(server_ms)
        results[pipeline] = (walls, servers, failures)

    print()
    print('=' * 96)
    print(f'목표 {GOAL_RAD} rad · group={args.group} · plan_only')
    print('=' * 96)
    for pipeline, (walls, servers, failures) in results.items():
        report(f'{pipeline} wall', walls)
        report(f'{pipeline} server', servers)
        if failures:
            print(f'{pipeline:22s}  실패 {len(failures)}회: {", ".join(failures)}')
        print('-' * 96)

    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
