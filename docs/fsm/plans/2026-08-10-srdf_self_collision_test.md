<!-- meta
updated: 2026-08-10
status:  실행 대기 — 사용자가 내일 실기로 확인 예정
owns:    그리퍼 SRDF 자기충돌 4쌍 누락 판별 실험 절차
-->

# 계획 — 그리퍼 SRDF 자기충돌 판별 실험

배경·원인 분석은 [[ws/cobot2/context/movegroup_rmpflow_review]] 3절이 단일 출처. 여기는
**실행 절차만** 둔다.

## 목적
`m0609_rg2.srdf`에 빠진 좌↔우 교차쌍 4개(`left_inner_knuckle↔right_outer_knuckle` 등)가
"닫힌 자세 메시 겹침" 때문인지, 자세와 무관한 자기충돌인지 공짜로 판별한다.

## 전제 조건 (실행 전 반드시 확인)
- 🔴 **`[OnRobot Modbus]: Connection failed!`가 T2 로그에 있으면 이 실험은 무효다.**
  `rg2_finger_joint`가 XRDF에서 lock되어 있어(`config/testcommand.md:206-207`), Modbus 연결이
  안 되면 그리퍼를 손으로 열어도 `/joint_states`에 반영이 안 될 수 있다(# UNVERIFIED — 이 ws에서
  실기 확인된 적 없음, 아래 2단계에서 직접 확인).

## 절차
1. `config/testcommand.md` 1~7절 순서로 T1(카메라)~T7(move_group, cumotion:=true) 기동.
2. 그리퍼 통신 확인:
   ```bash
   ros2 topic echo /joint_states --once | grep -A1 rg2_finger_joint   # 기대: ~0.757 rad (닫힘)
   ```
3. 그리퍼 열기:
   ```bash
   ros2 service call /onrobot/sendCommand onrobot_rg_msgs/srv/SetCommand "{command: 'o'}"
   ```
4. **재확인 — 여기서 막히면 3단계 이전으로 못 감**:
   ```bash
   ros2 topic echo /joint_states --once | grep -A1 rg2_finger_joint
   # 기대: 하한(-0.558505) 근처, 실측 -0.4793 rad (2026-08-11 확인).
   # 🔴 "~0 rad"가 아니다 — 조인트 하한(m0609_with_rg2.urdf:460)이 -0.558505이고
   #    닫힘(0.757)의 반대쪽 극단이 열림이다. 이전 기대값(~0)은 실기 확인 없이 적힌 오기.
   ```
   0.757 근처(닫힘)에 그대로면 Modbus 연결부터 고친다. 이 상태로 아래를 진행해도 무효.
5. T7(move_group) 재시작 — 큐가 안 비워지므로 이전 실험 잔재 제거.
6. 원인을 재현했던 것과 동일한 명령 실행(**`goal_setter_replan.py`**, `plan_only=False`라야
   재검증이 실제로 걸린다 — `reactive_replan.py`는 `plan_only=True`라 이 재검증을 안 거친다):
   ```bash
   ros2 run cumotion goal_setter_replan --ros-args -p vel_scale:=0.15
   ```
   첫 실행은 `vel_scale:=0.15`, 비상정지 버튼에 손 올린 채.
7. **T7(move_group) 터미널 로그**에서 `Found a contact between 'rg2_...'` 재등장 여부 확인
   (호출 스크립트 stdout이 아니다).

## 판정
- 경고 사라짐 → "닫힌 자세 메시 겹침" 확정. SRDF 안 건드리고 그립 전/후에만 재계획하는 식으로 우회.
- 경고 그대로 → 자세 무관 자기충돌 가능성. [[ws/cobot2/context/movegroup_rmpflow_review]] 3절의
  해결 절차(1: 판별 완료 상태 → 2: 실사용 range로 재샘플링 → 3: 최소침습 4쌍 추가+FCL 스윕)로.
