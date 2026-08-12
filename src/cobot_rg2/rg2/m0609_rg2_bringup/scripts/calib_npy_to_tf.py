#!/usr/bin/env python3
"""corecode 캘리브 결과(.npy, 4x4, mm) → ROS static_transform_publisher 인자(m).

사용:
  python3 scripts/calib_npy_to_tf.py corecode/Calibration_Tutorial/T_cam2base.npy base_link camera_link
  python3 scripts/calib_npy_to_tf.py T_gripper2camera.npy link_6 camera_link_webcam

npy는 "child가 parent 좌표계에서 갖는 pose"(= child 좌표를 parent 좌표로 옮기는 행렬)여야 한다.
  eye-to-hand  T_cam2base.npy      : parent=base_link, child=camera_link
  eye-in-hand  T_gripper2camera.npy: parent=link_6(또는 tool0), child=camera_link_webcam

⚠️ 좌표 규약 변환을 여기서 한다. npy는 cv2 출력이라 **OpenCV optical 규약**(z=전방, x=우, y=하)인데,
ROS의 camera_link는 REP-103(**x=전방**, y=좌, z=상)이다. 그냥 발행하면 90° 짝만큼 통째로 돌아가서
포인트클라우드가 로봇 옆에 엉뚱하게 떨어진다(2026-08-02 실제로 겪음. 지문: 출력 RPY의 roll ≈ -90°).
--no-optical 을 주면 변환을 생략한다(npy가 이미 body 규약일 때만).

프레임 이름 주의: m0609_rg2_bringup의 robot_state_publisher는 m0609_with_rg2.urdf.xacro를
쓰므로 TF에 나오는 이름은 base_link / link_1..link_6 / tool0 이다. dsr_description2 쪽
이름(base_0, link6)은 ros2_control용 URDF에만 있고 TF에는 발행되지 않는다.
"""
import sys
import numpy as np
from scipy.spatial.transform import Rotation

MM_TO_M = 1e-3


# ROS 광학 프레임 규약(REP-103): camera_link -> *_optical_frame = RPY(-90,0,-90).
# RealSense 고유값이 아니라 ROS 전체 공통이라 상수로 둔다.
R_LINK_TO_OPTICAL = Rotation.from_euler("xyz", [-90, 0, -90], degrees=True).as_matrix()


def npy_to_tf_args(T, parent, child, optical=True):
    R = T[:3, :3]
    # sqrtm(eye2hand) 출력이 정확히 직교가 아닐 수 있어 검사한다. 어긋나면 캘리브 자체가 나쁜 것.
    orth = np.abs(R @ R.T - np.eye(3)).max()
    det = np.linalg.det(R)
    if orth > 1e-6 or abs(det - 1.0) > 1e-6:
        print(f"⚠️ 회전행렬이 직교가 아님: max|RR^T-I|={orth:.2e}, det={det:.6f}", file=sys.stderr)
    if orth > 1e-3:
        raise SystemExit("회전행렬 오차가 1e-3을 넘는다. 캘리브 데이터를 다시 수집할 것.")
    # ponytail: 미세 오차만 SVD로 정규화. 큰 오차는 위에서 이미 중단시킨다.
    u, _, vt = np.linalg.svd(R)
    R_ortho = u @ vt
    if optical:
        R_ortho = R_ortho @ R_LINK_TO_OPTICAL.T  # optical 규약 → camera_link(body) 규약
    q = Rotation.from_matrix(R_ortho).as_quat()  # x y z w — ROS 순서와 동일
    t = T[:3, 3] * MM_TO_M
    return t, q


def main(argv):
    optical = "--no-optical" not in argv
    argv = [a for a in argv if a != "--no-optical"]
    if len(argv) != 3:
        raise SystemExit(__doc__)
    path, parent, child = argv
    t, q = npy_to_tf_args(np.load(path), parent, child, optical=optical)
    args = f"{t[0]:.6f} {t[1]:.6f} {t[2]:.6f} {q[0]:.8f} {q[1]:.8f} {q[2]:.8f} {q[3]:.8f}"
    rpy = Rotation.from_quat(q).as_euler("xyz", degrees=True)
    print(f"# {path}: 평행이동 {np.linalg.norm(t)*1000:.1f} mm, "
          f"optical변환={'적용' if optical else '생략'}, RPY(deg)={np.round(rpy, 1)}")
    if optical and abs(abs(rpy[0]) - 90) < 15:
        print("# ⚠️ roll이 아직 ±90° 근처다. npy가 이미 body 규약이었을 수 있다 → --no-optical 로 비교해볼 것.",
              file=sys.stderr)
    print(f"ros2 run tf2_ros static_transform_publisher {args} {parent} {child}")


def _selfcheck():
    """알려진 변환을 넣어 왕복이 맞는지 확인한다."""
    R = Rotation.from_euler("ZYZ", [30, 45, 60], degrees=True)
    T = np.eye(4)
    T[:3, :3] = R.as_matrix()
    T[:3, 3] = [100.0, -250.0, 500.0]  # mm
    t, q = npy_to_tf_args(T, "a", "b", optical=False)
    assert np.allclose(t, [0.1, -0.25, 0.5]), t
    assert np.allclose(Rotation.from_quat(q).as_matrix(), R.as_matrix(), atol=1e-9)

    # optical=True는 정확히 광학규약 회전만큼 되돌려야 한다:
    # optical 규약의 +z(전방)가 body 규약의 +x(전방)로 옮겨가는지로 확인한다.
    _, q_opt = npy_to_tf_args(T, "a", "b", optical=True)
    R_body = Rotation.from_quat(q_opt).as_matrix()
    assert np.allclose(R_body @ [1, 0, 0], R.as_matrix() @ [0, 0, 1], atol=1e-9), "optical 변환이 틀렸다"
    assert np.allclose(R_body, R.as_matrix() @ R_LINK_TO_OPTICAL.T, atol=1e-9)

    # 직교성이 깨진 입력은 거부되어야 한다
    bad = T.copy()
    bad[:3, :3] *= 1.01
    try:
        npy_to_tf_args(bad, "a", "b")
    except SystemExit:
        pass
    else:
        raise AssertionError("직교성 검사가 동작하지 않는다")
    print("selfcheck OK")


if __name__ == "__main__":
    if len(sys.argv) == 2 and sys.argv[1] == "--selfcheck":
        _selfcheck()
    else:
        main(sys.argv[1:])
