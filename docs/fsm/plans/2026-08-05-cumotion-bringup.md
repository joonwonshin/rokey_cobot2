<!-- meta
updated: 2026-08-06 12:40
status:  live
owns:    cuMotion/nvblox 브링업 게이트·빌드 절차 · nvblox 실행 절차 본체(§6) · task_manager 백엔드 경계 설계(§5-1)
-->

# cuMotion / nvblox 브링업 명령서 (2026-08-05)

> 목적: Isaac ROS 3.2 컨테이너에서 cuMotion을 M0609+RG2로 띄우고, **OMPL 대비 계획 시간**을 잰다.
> 그 숫자가 [[ws/cobot2/plans/2026-08-05-foundationpose-graspgenx-pick]]의 (a) 계획시점 우회 /
> (b) 실행중 stop→재계획 중 무엇으로 갈지를 정한다. 추측 말고 여기서 측정한다.
>
> ⚠️ 아래 명령 중 **컨테이너 안에서 실제로 실행해 검증한 것은 하나도 없다.** 전부 소스를 읽고 구성한 것이다.
> 실패하면 그 자리에서 에러를 기록하고 이 문서를 고친다.

---

## 0. 호스트에 미리 배치해 둔 것 (2026-08-05 완료)

컨테이너는 **`~/cobot2_ws/isaac_ros-dev` 하나만** 마운트한다(`run_dev.sh:288`
`-v $ISAAC_ROS_DEV_DIR:/workspaces/isaac_ros-dev`). `~/cobot2_ws/src`는 **안 보인다.**
그래서 필요한 파일을 마운트 안쪽으로 복사해 뒀다.

| 호스트 경로 | 컨테이너 경로 | 용도 |
|---|---|---|
| `isaac_ros-dev/m0609/m0609_kinematics.urdf` | `/workspaces/isaac_ros-dev/m0609/m0609_kinematics.urdf` | **cuRobo용.** visual/collision 34개를 제거해 `package://` 0개 — 컨테이너에 `dsr_description2`가 없어도 파싱된다. 충돌 형상은 XRDF의 구에서 온다 |
| `isaac_ros-dev/m0609/m0609_with_rg2.urdf` | 〃 | 전체 URDF(메시 포함). 폴백용 |
| `isaac_ros_cumotion_robot_description/xrdf/m0609_rg2.xrdf` | 〃 | ⚠️ **cuMotion은 XRDF를 임의 경로가 아니라 이 패키지의 `xrdf/`에서 파일명으로 찾는다** (`update_kinematics.py:62-64`). 그래서 여기 복사했다 |

> **심링크 금지.** `~/cobot2_ws/src`를 가리키는 심링크는 컨테이너 안에서 대상 경로가 없어 깨진다. 반드시 복사한다.
>
> XRDF 정본은 `src/cobot_rg2/rg2/m0609_rg2_moveit/config/m0609_rg2.xrdf`다. 고치면 위 두 곳에 다시 복사한다.

---

## 1. 게이트 A — 컨테이너가 GPU를 보는가

컨테이너 진입 후:

```bash
nvidia-smi                       # RTX 4060이 나와야 한다
ls /workspaces/isaac_ros-dev/m0609/   # urdf 2개 + xrdf 가 보여야 한다
export ROS_DOMAIN_ID=93          # ⚠️ 컨테이너 안에서 매번. --network host라 밖 노드와 통신된다
ros2 node list                   # 호스트에서 bringup을 띄워 뒀다면 /dsr01/* 가 보인다
```

**통과 못 하면 여기서 멈춘다.** GPU가 안 보이면 아래는 전부 무의미하다.

---

## 2. 게이트 B — 빌드

### 2-0. 선행 3가지 (2026-08-05 실기에서 전부 걸렸다)

**① cuRobo는 git 서브모듈이다 — 따로 받아야 한다.**
호스트에서 `--depth 1 --branch v3.2-14`로 클론했기 때문에 서브모듈이 비어 있다.
`git submodule status` 앞에 `-`가 붙어 있으면 미초기화 상태다.

```bash
cd /workspaces/isaac_ros-dev/src/isaac_ros_cumotion
git submodule update --init --recursive     # curobo_core/curobo 를 받는다
```

**② `isaac_ros-dev/` 루트에 `COLCON_IGNORE`가 있다 → colcon이 아무 패키지도 못 본다.**
호스트 워크스페이스(`~/cobot2_ws`) 빌드가 Isaac 패키지를 집지 않게 하려고 둔 파일인데,
마운트로 컨테이너에도 그대로 보인다. **지우면 안 된다**(호스트 빌드가 오염된다).
대신 `--base-paths src`로 한 단계 아래에서 스캔한다.

**③ `src/isaac_ros_common.bak/`가 모든 패키지를 중복시킨다** → colcon이 중복 패키지명으로 실패.
**2026-08-05에 조사 후 삭제했다.** 조사 결과(다시 만들지 않기 위해 기록):
- 내용은 `isaac_ros_common` @ `v3.2-14`와 **동일한 커밋**(`fcf4d9e`), 고유 파일은 3개뿐이었다
- `docker/Dockerfile.{x86_64,aarch64}.lightninglink` — 커스터마이징이 **아니라 복사 사고**다.
  원본은 `Dockerfile.x86_64 -> Dockerfile.base` **심링크**인데, `.bak` 쪽은 "Dockerfile.base"라는
  **문자열이 든 15바이트 일반 파일**이었다(심링크 미보존 복사). 쓸모없다
- `isaac_ros_common/scripts/.isaac_ros_common-config` — **이것만 의미가 있었다.** 아래에 옮겨 적는다

### 컨테이너 이미지에 RealSense 레이어를 넣는 법 (지금은 안 들어가 있다)

`run_dev.sh:37`의 기본값은 `IMAGE_KEY=ros2_humble`이라 현재 이미지는 **`x86_64.ros2_humble`**,
즉 **컨테이너 안에 RealSense 드라이버가 없다**. 넣으려면 `run_dev.sh`와 같은 디렉토리에:

```bash
echo 'CONFIG_IMAGE_KEY=ros2_humble.realsense' \
  > src/isaac_ros_common/isaac_ros_common/scripts/.isaac_ros_common-config
```

(`run_dev.sh:27-28`이 이 파일을 source하고 `:40-41`이 `IMAGE_KEY`를 덮는다.
사용 가능한 레이어: `base`, `realsense`, `ros2_humble`.)

> **현재 구성에서는 필요 없다.** RealSense 드라이버는 **호스트**에서 돌고(`reals`),
> 컨테이너는 `--network host`로 `/camera/*` 토픽을 구독만 하면 된다. nvblox도 드라이버가
> 아니라 depth **토픽**을 먹는다. 컨테이너 안에서 드라이버를 직접 띄워야 할 때만 위 설정을 쓴다
> (예: 호스트가 없는 클라우드 GPU — `[[ws/cobot2/plans/2026-08-04-gpu-rental-checklist]]`).

### 2-1. 빌드

```bash
cd /workspaces/isaac_ros-dev
colcon build --symlink-install --base-paths src \
  --packages-up-to isaac_ros_cumotion_moveit isaac_ros_cumotion_robot_description
source install/setup.bash
```

실제로 빌드되는 것 8개(위상순, 2026-08-05 확인):
`isaac_ros_common` → `curobo_core` → `isaac_ros_cumotion_interfaces` →
`isaac_ros_cumotion_python_utils` → `isaac_ros_cumotion_robot_description` →
`nvblox_msgs` → `isaac_ros_cumotion` → `isaac_ros_cumotion_moveit`

`curobo_core`가 CUDA 커널을 컴파일하므로 **처음엔 오래 걸린다**(수십 분 각오).

```bash
python3 -c "import curobo; print('curobo OK', curobo.__file__)"
```

### 2-2. ⛔ `curobo_core` 빌드 실패 — `std::lerp` 충돌 (2026-08-05 발생·해결)

```
helper_math.h:1130: error: 'float lerp(float, float, float)' conflicts with a previous declaration
/usr/include/c++/11/cmath:1911: note: previous declaration 'constexpr float std::lerp(float, float, float)'
```

**원인은 CUDA가 아니라 C++ 표준이다.** C++20이 `std::lerp`를 추가했고 libstdc++의 `<math.h>`가
그것을 전역 네임스페이스로 주입한다. torch `cpp_extension`이 `-std=c++20`을 강제하므로
(cuRobo `setup.py`엔 `-std` 지정이 없다 — 확인함) cuRobo의 전역 `lerp`와 충돌한다.
cuRobo 커밋 `36ea382`는 2024-11-22자로 이 조합보다 오래됐다.

**해결:** 그 스칼라 `lerp`는 **cuRobo 전체에서 한 번도 호출되지 않는다**(grep 확인).
NVIDIA `helper_math.h` 샘플에서 딸려온 죽은 코드다. `#if __cplusplus < 202002L`로 감쌌다.
`float2/3/4` 오버로드는 인자 타입이 달라 충돌하지 않으므로 건드리지 않았다.

⚠️ **이 파일은 git 서브모듈(`curobo_core/curobo`)이라 재-init하면 패치가 사라진다.**
`patches/curobo-helper_math-cpp20-lerp.patch`에 저장해 뒀다. 날아가면:

```bash
cd /workspaces/isaac_ros-dev/src/isaac_ros_cumotion/curobo_core/curobo
git apply ~/cobot2_ws/patches/curobo-helper_math-cpp20-lerp.patch   # 호스트 경로 기준
```

**재빌드: `build/`를 지울 필요 없다.** (이전 판에 "복사되므로 지워야 한다"고 적었던 것은 **틀렸다**.)
`--symlink-install`이라 colcon은 소스를 복사하지 않고 **심링크**한다 — 2026-08-05 실측:

```
build/curobo_core/curobo/src/curobo
  -> /workspaces/isaac_ros-dev/src/isaac_ros_cumotion/curobo_core/curobo/src/curobo
```

즉 패치가 즉시 반영되고, ninja가 헤더 mtime 변화를 보고 알아서 재컴파일한다.
그냥 다시 돌리면 된다:

```bash
cd /workspaces/isaac_ros-dev
colcon build --symlink-install --base-paths src \
  --packages-up-to isaac_ros_cumotion_moveit isaac_ros_cumotion_robot_description
```

그래도 이상하면 그때 `rm -rf build/curobo_core install/curobo_core`로 초기화한다
(단 CUDA 커널을 처음부터 다시 컴파일하므로 수십 분을 다시 쓴다 — 먼저 그냥 재빌드해 볼 것).

> 대안(미시도): `-std=c++17`을 강제하는 방법도 있으나, torch가 c++20으로 빌드돼 있으면
> 헤더 호환이 깨질 수 있다. 죽은 코드 한 덩이를 막는 쪽이 blast radius가 작다.

> ⚠️ **`--base-paths src`를 빼먹으면** `Package '...' specified with --packages-up-to was not found`가
> 뜬다. 패키지가 없는 게 아니라 루트 `COLCON_IGNORE` 때문에 **스캔 자체를 안 한 것**이다.

### 2-3. ⛔ 런타임 실패 — `module 'warp' has no attribute 'torch'` (2026-08-05 발생)

게이트 D 첫 실행에서 `load_motion_gen()` 안에서 죽는다:

```
world_mesh.py:67  self._wp_device = wp.torch.device_from_torch(self.tensor_args.device)
warp/__init__.py:603 in __getattr__
AttributeError: module 'warp' has no attribute 'torch'
```

**2-2와 같은 종류의 문제다 — 의존성 버전 상한이 없어서 생긴 어긋남.**

- 컨테이너에 깔린 warp: **1.16.0** (실측)
- cuRobo 커밋: `36ea382` **2024-11-22**. 버전 게이트가 `1.2.1`까지밖에 모른다(`util/warp.py:60`)
- 원인: cuRobo `setup.cfg:53`이 `warp-lang>=0.9.0`으로 **상한을 안 걸었다.**
  cuRobo 자신의 dockerfile도 `pip3 install warp-lang`을 핀 없이 부른다
  (`docker/aarch64.dockerfile:137`) → pip이 최신을 끌어왔다. **14개 마이너 버전 드리프트.**

#### ❌ 안 되는 해결: `import warp.torch` 추가

처음에 `world_mesh.py`에 명시 임포트를 넣어 봤으나 **warp 1.16.0에는 `warp.torch` 모듈이
아예 없다** — `AttributeError`가 `ModuleNotFoundError`로 바뀔 뿐이다. 패치는 남겨 뒀지만
`try/except ImportError`로 삼키게 해서, 원래 코드와 같은 지점(67행)에서 죽도록 무해화했다
(`patches/curobo-warp-torch-import.patch`). **다운그레이드 후에는 이 임포트가 정상 동작한다.**

#### ✅ 해결: warp 다운그레이드

```bash
pip3 install 'warp-lang==1.5.0'
```

- 1.5.0은 cuRobo 커밋과 **동시대**(2024-12)이고 `>1.2.1`이라 `warp_support_kernel_key` 게이트가
  최신 경로를 탄다
- **colcon 재빌드 불필요.** warp 커널은 warp 자체 JIT 캐시(`~/.cache/warp`)라 첫 실행에서
  몇 분 걸릴 뿐, `curobolib`의 CUDA 확장(2-2에서 컴파일한 것)과는 무관하다
- 컨테이너 안에서 warp를 쓰는 건 **cuRobo뿐**이다 (`src/` 전수 grep: 다른 소비자는
  `GraspGenX/end2end/dynamic_playback.py` 하나인데 그건 호스트 `uv` 트랙이라 무관)
- ⚠️ **이미지 밖 변경이라 컨테이너를 새로 만들면 날아간다.** 재현 절차를 §0에 적어 둘 것

1.5.0에서도 깨지면 1.4.2로 한 칸 더 내린다. 그래도 안 되면 warp 최신 API 위치를 찾는다:

```bash
python3 -c "
import warp, pkgutil
print('warp', warp.config.version, warp.__file__)
print('submodules:', [m.name for m in pkgutil.iter_modules(warp.__path__)])
print('top-level device_from_torch:', hasattr(warp, 'device_from_torch'))
"
```

---

## 3. 게이트 C — cuRobo가 우리 XRDF를 읽는가 ⭐

**로봇도 nvblox도 필요 없다. 가장 값싸고 가장 중요한 검증이다.**
XRDF가 틀렸으면 여기서 죽고, 아래 단계를 아무리 해도 안 된다.

### 3-1. XRDF 파싱 (2026-08-05 통과 ✅)

```bash
python3 - <<'EOF'
from curobo.cuda_robot_model.util import load_robot_yaml
from curobo.types.file_path import ContentPath

cp = ContentPath(
    robot_xrdf_absolute_path='/workspaces/isaac_ros-dev/m0609/m0609_rg2.xrdf',
    robot_urdf_absolute_path='/workspaces/isaac_ros-dev/m0609/m0609_kinematics.urdf',
)
cfg = load_robot_yaml(cp)
print('XRDF 로드 OK')
k = cfg['robot_cfg']['kinematics']
print('  base   :', k.get('base_link'))
print('  ee     :', k.get('ee_link'))
print('  joints :', k.get('cspace', {}).get('joint_names'))
EOF
```

실기 결과 — `base_link` / `tool0` / `joint_1..6 + rg2_finger_joint`.

> **`rg2_finger_joint`가 7번째로 찍히는 건 정상이다. 계획 DOF가 7이 된 게 아니다.**
> `xrdf_utils.py:136`이 `all_joint_names = active_joints + lock_joints`로 합쳐서 찍기 때문이다.
> XRDF `cspace.joint_names`는 6개뿐이고, `rg2_finger_joint`는 `default_joint_positions`의
> `-0.558505`(gripper_open)로 **lock** 된다(`:126-128`). RG2의 나머지 5개 관절은 URDF mimic이라
> `get_controlled_joint_names()`에 애초에 안 들어오고, cuRobo가 mimic을
> 자체 처리한다(`urdf_kinematics_parser.py:166-202`). → **cuMotion은 6 DOF로 계획한다.**

### 3-2. 정기구학 + 구 검증 ← **지금 여기**

스크립트는 호스트에 두었다(바인드 마운트라 컨테이너에서 그대로 보인다):
`isaac_ros-dev/m0609/gate_c.py`

```bash
cd /workspaces/isaac_ros-dev && source install/setup.bash
python3 m0609/gate_c.py
```

**판정 (기대값은 스크립트가 직접 대조해 합/불을 찍는다):**

| 항목 | 기대 | 근거 |
|---|---|---|
| 계획 DOF | **6** | XRDF `cspace.joint_names` |
| 충돌 구 개수 | **75** | XRDF `geometry:` 절 실측 (base 10, link_1..6 = 4/5/4/6/5/4, RG2 37) |
| all-zeros `tool0` 위치 | **[0.0001, 0.0064, 1.0345] m** | ↓ |
| 자기충돌 무시쌍 | **34** | SRDF `disable_collisions` |

> EE 기대값은 `tf2_echo`가 아니라 **URDF 관절 origin을 직접 곱해 호스트에서 따로 계산한 값**이다
> (`joint_1..6` 전부 0일 때 체인이 수직으로 서서 z=1.0345 m). cuRobo와 무관한 기준점이라
> 대조에 쓸 수 있다. `tf2_echo base_link tool0`은 **현재 자세**를 주므로, 로봇이 전자세(all-zeros)에
> 있지 않으면 이 값과 안 맞는 게 당연하다 — 그걸로 판정하지 말 것.

**불합격일 때:**
- 구 개수 ≠ 75 → XRDF `geometry:` 절이 안 읽힌 것 (`collision.geometry` 이름 오타 의심)
- EE 오차 > 1 mm → URDF/XRDF의 base_link·tool0 지정이 어긋난 것
- 반지름이 이상하게 크면 `scripts/fit_spheres.py`를 다시 돌린다

---

## 4. 게이트 D — cuMotion 플래너 노드 (nvblox 없이) ← **지금 여기**

`read_esdf_world`가 **기본 False**라 nvblox 없이 MoveIt planning scene(=기존 octomap)을 쓴다.
**두 개를 동시에 켜지 않는다** — 실패 시 원인 분리가 안 된다.

### 4-0. ⛔ 먼저 컨테이너 안에서 이것부터 확인한다

```bash
echo "ROS_DOMAIN_ID=[$ROS_DOMAIN_ID]"
```

**비어 있으면 노드는 뜨지만 로봇을 못 본다.** `run_dev.sh:230`이 `-e ROS_DOMAIN_ID`로
**호스트 환경변수를 그대로 넘기는데**, 이 랩탑의 `~/.bashrc`는 `rdm` alias를 쳐야만
`ROS_DOMAIN_ID=93`을 설정한다. `rdm` 없이 연 터미널에서 `run_dev.sh`를 띄웠으면 컨테이너는
도메인 0이다. 컨테이너 재시작 없이 그 안에서 고칠 수 있다:

```bash
export ROS_DOMAIN_ID=93     # 이후에 띄우는 노드에만 적용된다
ros2 topic echo /joint_states --once   # 7개 관절이 나와야 정상
```

`/joint_states`는 호스트의 `joint_state_publisher`가
`/dsr01/joint_states` + `/gripper_joint_states`를 합쳐 내보내는 토픽이다
(`bringup.launch.py:175-179`). 플래너 노드의 기본값이 `/joint_states`라 **리맵이 필요 없다.**

### 4-1. 실행

```bash
cd /workspaces/isaac_ros-dev && source install/setup.bash
ros2 run isaac_ros_cumotion cumotion_planner_node --ros-args \
  -p robot:=m0609_rg2.xrdf \
  -p urdf_path:=/workspaces/isaac_ros-dev/m0609/m0609_kinematics.urdf \
  -p read_esdf_world:=False \
  -p publish_curobo_world_as_voxels:=True \
  -p voxel_size:=0.02 \
  -p publish_voxel_size:=0.02
```

파라미터 이름은 **소스에서 확인했다**(`cumotion_planner.py:62-95`). 주의할 점:

- `robot:=` 은 **파일명만** 준다(경로 아님, §0 참고)
- **`tool_frame`은 주지 않는다.** 안 주면 XRDF `tool_frames[0]`(=`tool0`)을 쓰는데
  (`:131-136`, `:330`), SRDF `manipulator` 그룹의 `tip_link`도 `tool0`이라 이미 일치한다.
  불일치하면 `:771`이 `relaunch node with tool_frame = ...` 로 알려준다
- `voxel_size`(계획용)와 `publish_voxel_size`(시각화용)는 **별개 파라미터**다. 둘 다 기본 0.05
- **뜨는 데 시간이 걸린다.** `load_motion_gen()` → `warmup()`이 액션 서버보다 먼저 돈다(`:260-261`).
  cuRobo 커널 워밍업이라 첫 실행은 수십 초 걸릴 수 있다 — 멈춘 게 아니다

### 4-2. 합격 판정 (2026-08-05 통과 ✅)

```bash
ros2 action list | grep cumotion      # /cumotion/move_group
```

실기 로그:
```
[INFO] warming up cuMotion, wait until ready
[INFO] cuMotion is ready for planning queries!     ← 워밍업 1.7초
```

- 액션 이름은 `cumotion/move_group`, 타입 `moveit_msgs/action/MoveGroup` (`:279`)
- `opt_base.py:298`의 sparse tensor UserWarning은 **무시해도 된다** (torch 내부 경고)
- 이 단계에서 로봇은 **움직이지 않는다.** move_group이 이 액션을 부르기 전까지는 대기만 한다

> **`/curobo/voxels`에 메시지가 안 오는 것은 정상이다.** 이 퍼블리시는 `execute_callback`
> 안에서만 일어난다(`:665 → :594 → :622`). **계획 요청이 와야 나온다.** 게다가
> `get_subscription_count() > 0`일 때만 계산한다(`:623`). 대기 중 `ros2 topic hz`로
> 판정하려던 이전 판의 기준은 **틀렸다** — 게이트 E에서 첫 계획을 돌린 뒤에 본다.

---

## 4-3. 🔴 여기서 발견한 것 — **cuMotion은 octomap을 아예 안 본다**

`read_esdf_world:=False`일 때 cuMotion이 세계를 받는 경로는 **한 곳뿐**이다:

```python
# cumotion_planner.py:662-665
scene = goal_handle.request.planning_options.planning_scene_diff
world_objects = scene.world.collision_objects        # ← collision_objects "만"
world_update_status = self.update_world_objects(world_objects)
```

`cumotion_planner.py` 전체에 **`octomap` 문자열이 0건**이다(grep 확인).
MoveIt 플러그인은 `getPlanningSceneMsg()`로 **전체** 씬을 보내주는데
(`cumotion_move_group_client.cpp:72,81`), 받는 쪽이 `world.octomap` 필드를 그냥 버린다.

### 이게 이 프로젝트에 갖는 의미

| | OMPL (현재) | cuMotion + `read_esdf_world:=False` | cuMotion + nvblox |
|---|---|---|---|
| 테이블·박스(CollisionObject) | 본다 | 본다 | 본다 |
| **사람 팔 (octomap 복셀)** | **본다** | **❌ 못 본다** | 본다 (ESDF) |

**우리 프로젝트의 목적(사람 팔 우회)에 직결된다.** RealSense가 만드는 사람 팔은
지금 octomap 복셀로만 존재하므로, 게이트 E에서 cuMotion으로 전환하면
**계획은 성공하는데 사람 팔을 통과하는 궤적이 나온다.**

⚠️ **가장 위험한 실패 방식이다 — 성공처럼 보인다.** 계획 시간은 빨라지고 에러도 안 나므로,
"cuMotion이 더 빠르다"는 결론만 남고 장애물을 안 봤다는 사실은 드러나지 않는다.

### 따라서 계획 수정

- **게이트 F(nvblox)는 선택이 아니라 필수다.** 이전 판이 "게이트 E 통과 후"의 부가 단계로
  적어 둔 것은 틀렸다. cuMotion이 미모델링 장애물을 보는 **유일한** 경로다
- 게이트 E는 **속도 비교 전용**으로만 쓴다(§7의 OMPL vs cuMotion 계획 시간).
  **이 단계에서 사람 팔을 넣고 실기를 돌리지 않는다**
- 게이트 E 중 실기 검증이 필요하면, 장애물을 **명시적 CollisionObject**로 넣어야 한다
  (계획서 `2026-08-05-foundationpose-graspgenx-pick.md` Phase 0-G의 ACM/CollisionObject 작업과
  같은 배선이다 — 중복 작업 아님)

---

## 5. 게이트 E — MoveIt 파이프라인에 붙이기 ✅ **2026-08-06 통과 (로봇 없이)**

> **결과 요약**
> - E-1(컨테이너에 `~/cobot2_ws` 추가 마운트 + 우리 패키지 빌드) 채택·성공. 5개 패키지 빌드 통과
>   (`dsr_description2`·`onrobot_rg_description`·`onrobot_rg_msgs`·`m0609_rg2_bringup`·`m0609_rg2_moveit`)
> - `moveit.launch.py`에 `cumotion:=true` 인자 추가 → OMPL·cuMotion 두 파이프라인 공존 확인
> - **OMPL vs cuMotion 계획시간 실측 완료** → §7
> - ⚠️ **로봇·카메라 없이 잰 것이다.** `standalone:=true`, `octomap:=false`, 빈 세계.
>   실기 수치가 아니라 **배선이 살아 있다는 증거 + 빈 세계 기준선**이다.
> - 가는 길에 실측으로 걸린 2가지(joint_states velocity, XRDF 구 과대추정)는
>   [[ws/cobot2/context/constraints]]가 단일 출처다. 아래 5-2에 요약만 둔다.

### 5-0. 실행 절차 (2026-08-06 실측한 그대로)

```bash
# 호스트에서 컨테이너 기동 — ~/cobot2_ws 를 추가 마운트한다 (E-1)
./run_dev.sh -a "-v $HOME/cobot2_ws:/workspaces/cobot2_ws"
```

컨테이너 안에서, **셸마다** 매번:

```bash
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
source /workspaces/cobot2_ws/install_container/setup.bash
export ROS_DOMAIN_ID=93
```

**① 우리 패키지 빌드** — ⚠️ `build/`·`install/`이 아니라 `build_container/`·`install_container/`다.
호스트와 컨테이너는 워크스페이스 경로가 다른데(`/home/kimkh/cobot2_ws` vs `/workspaces/cobot2_ws`)
`install/setup.bash`에는 **절대경로가 박힌다.** 같은 디렉토리를 쓰면 한쪽이 반드시 깨진다.
(둘 다 `.gitignore`에 추가해 뒀다.)

```bash
cd /workspaces/cobot2_ws
colcon build --symlink-install --base-paths src \
  --build-base build_container --install-base install_container \
  --packages-up-to m0609_rg2_moveit
```

`--base-paths src`가 필요한 이유는 §2-0 ②와 같다(루트 `COLCON_IGNORE`).
MoveIt·RViz·controller_manager는 Isaac ROS 3.2 이미지에 **이미 apt로 들어 있다**(2026-08-06 확인).

**② 플래너 노드** (§4-1과 동일, `read_esdf_world:=False`)

**③ move_group + RViz**

```bash
ros2 launch m0609_rg2_moveit moveit.launch.py cumotion:=true
#   실기에 얹을 때는 standalone:=false (bringup 위에 얹는다)
#   빈 세계 기준선만 잴 때는 standalone:=true octomap:=false rviz:=false
```

로그에서 두 줄이 다 나와야 합격이다:

```
Loading planning pipeline 'ompl'                 → Using planning interface 'OMPL'
Loading planning pipeline 'isaac_ros_cumotion'   → Using planning interface 'Generate minimum-jerk trajectories using NVIDIA Isaac ROS cuMotion'
```

**④ 측정** — RViz 드롭다운을 사람이 번갈아 누르는 대신 스크립트가 `pipeline_id`만 바꿔 부른다.
`plan_only=True` 고정이라 **로봇은 움직이지 않는다.**

```bash
python3 /workspaces/cobot2_ws/scripts/bench_planning_time.py --repeat 20
```

### 5-1. ⚠️ 함정 3개 (전부 2026-08-06에 실제로 걸렸다)

1. **MoveGroup 액션 이름은 `/move_action`이다.** `/move_group`이 아니다(그건 노드 이름).
2. **`/joint_states` publisher가 1개인지 먼저 확인한다** (`ros2 topic info /joint_states`).
   죽인 줄 알았던 옛 launch가 살아 있으면 velocity 있는/없는 메시지가 번갈아 와서
   계획이 **산발적으로만** 실패한다.
3. **`pkill -f "joint_state_publisher"`를 `docker exec bash -c`로 쓰지 말 것.**
   패턴이 **자기 명령줄에도 매칭돼 자기 셸을 먼저 죽인다** — 뒤 명령이 실행되지 않는데
   출력은 조용해서 "정리됐다"로 오독한다. `pgrep`으로 PID를 뽑아 `kill <pid>` 한다.

### 5-2. 가는 길에 걸린 것 (요약 — 정본은 constraints.md)

| 증상 | 원인 | 조치 |
|---|---|---|
| cuMotion 계획 10/10 `ERROR(-1)` | `/joint_states`에 velocity 배열이 없음 (`cumotion_planner.py:698-704`) | 두 launch에 `publish_default_velocities: True` |
| `INVALID_START_STATE_SELF_COLLISION` | XRDF 구가 실제 링크보다 뚱뚱해 6쌍이 겹침. **같은 자세를 OMPL은 통과** | 6쌍을 `self_collision.ignore`에 추가 (⚠️ 2쌍은 보호 포기 — 실기 전 재검토) |

진단 도구: `scripts/diag_self_collision.py` (링크쌍별 침투량을 이름으로 찍는다).

### 5-3. 원래 설계 (참고)

`ur.launch.py`가 보여주는 표준 방식은 **두 파이프라인 공존**이다:

```python
{'planning_pipelines': ['ompl', 'isaac_ros_cumotion']},
{'isaac_ros_cumotion': <isaac_ros_cumotion_moveit/config/isaac_ros_cumotion_planning.yaml 내용>}
```

→ **RViz MotionPlanning 패널의 플래너 드롭다운에서 OMPL ↔ cuMotion을 전환**할 수 있다.
같은 목표로 두 번 계획해 시간을 비교하는 게 이번 측정의 핵심이다.

`moveit.launch.py`의 `planning_pipelines` 딕셔너리(103-107행)에 추가하면 된다.

### ⚠️ 마운트 문제 — ✅ **해결됨(E-1 채택, 2026-08-06)**

`move_group`은 `m0609_rg2_moveit`(호스트 `~/cobot2_ws/src`)에 있는데 컨테이너에 마운트가 안 돼 있다.
플러그인(`isaac_ros_cumotion_moveit`)은 `moveit_msgs::action::MoveGroup`을 쓰는 **얇은 클라이언트**라
CUDA 의존이 없지만(`package.xml` 확인), 실행 구성은 둘 중 하나를 골라야 한다:

| 안 | 방법 | 평가 |
|---|---|---|
| **E-1 (권장)** | 컨테이너를 `~/cobot2_ws`까지 마운트해 다시 띄우고, 그 안에서 우리 패키지도 빌드해 move_group을 **컨테이너 안에서** 실행<br>`./run_dev.sh -a "-v $HOME/cobot2_ws:/workspaces/cobot2_ws"` | Isaac ROS 3.2 = Humble이라 우리 패키지가 빌드될 가능성이 높다. 구성이 단순 |
| E-2 | 호스트 move_group + 컨테이너 플래너 노드. `--network host`라 통신은 된다 | 플러그인 `.so`를 호스트에서 빌드해야 하는데 `isaac_ros_common` 빌드툴 의존이 걸린다. **미검증** |

컨테이너를 껐다 켜야 하므로, **게이트 C까지 끝낸 뒤에** E-1로 다시 띄우는 것을 권한다.

### 5-1. 앞으로 올 `task_manager`가 지켜야 할 경계 (설계 지침, 2026-08-06)

`task_manager`(음성 명령 → 동작 시퀀스, [[ws/cobot2/plans/2026-08-05-foundationpose-graspgenx-pick]] 소관)는
아직 코드가 없다. 그런데 이 문서만 봐도 이미 octomap→nvblox, MoveIt→cuMotion→(장차 cuRobo)로
백엔드 후보가 3중이다(§4-3, §6). 코드를 쓰기 전에 경계를 정해 두지 않으면, 백엔드를 바꿀 때마다
`task_manager`까지 고쳐야 하는 사태가 반복된다.

**규칙**: `task_manager`는 "장면을 준비해라"(`scene_prep(target_id) -> bool`)와
"이 포즈들로 계획·실행해라"(`plan_and_execute(pose_list) -> bool`) 딱 두 함수만 부른다.
그게 지금 MoveIt `CollisionObject`/`PlanningScene` API로 채워지는지, cuMotion 플래너
액션 호출로 채워지는지, 나중에 cuRobo world config로 채워지는지는 `task_manager`가 몰라야 한다.

- 이 규칙이 지켜지면, octomap→nvblox 교체나 cuMotion/cuRobo 전환은 **이 두 함수 내부만**
  바뀌는 일이다. 반대로 `move_group` 액션 이름이나 `CollisionObject` 메시지 포맷이 호출부에
  직접 나타나기 시작하면, 나중에 정면 재작성이 된다.
- GraspGenX 쪽(`grasp_selector.py`)은 이미 이 경계를 지키고 있다 — grasp 후보 생성·필터링이
  전부 GraspGenX 전용 포인트클라우드 충돌 검사이고, octomap도 nvblox도 모른다. `/grasp/best`
  포즈(`base_link` 프레임)만 `plan_and_execute()`의 입력 계약으로 넘기면 된다.
- 근거·출처: 팀 코드 감사(`code-audit-calib-gripper-backend.md`, 2026-08-05) §3.

---

## 6. 게이트 F — nvblox 켜기 🔴 **선택 아님, 필수** (근거: §4-3)

cuMotion이 **사람 팔을 보는 유일한 경로**다. octomap은 cuMotion에 전달되지 않는다.

> ✅ **`isaac_ros_nvblox` 빌드는 2026-08-06 완료.** 게이트 E(속도 비교)보다 먼저 착수했다 —
> E는 `~/cobot2_ws` 전체를 추가 마운트해야 하지만(§5 E-1) F(이 아래 명령들)는 지금 컨테이너
> 마운트(`isaac_ros-dev`만)로 충분해서 재시작 없이 바로 갈 수 있었다. 빌드 중 막힌 3가지
> (NITROS 코어 레포 미클론·rosdep magic_enum 미해석·업스트림 CMakeLists 29곳의
> `find_package(magic_enum)` 누락)와 패치 위치는 [[ws/cobot2/isaac_ros_nvblox_setup]] §5-1이 단일 출처다.
> ⚠️ **warp-lang 1.5.0 다운그레이드(§2-3)는 컨테이너 재시작으로 유실된 상태였다** — 재적용함
> (`pip3 install 'warp-lang==1.5.0'`, 2026-08-06).
>
> ✅ **게이트 F 자체도 2026-08-06 통과** — 아래 두 노드를 라이브 카메라로 동시 실행,
> `cuMotion is ready for planning queries!`(워밍업 ~5초) + `ros2 action list`에
> `/cumotion/move_group` 확인. **로봇은 안 움직였다** — 액션 서버가 뜬 것만 확인, 실제 계획
> 요청(move_group 경유)은 게이트 E(§5) 배선이 있어야 보낼 수 있다.

### 6-0. ⭐ 파이프라인 전체 그림 (2026-08-06 관통 확인)

```
호스트                                          컨테이너
─────────────────────────────────────────────────────────────────────────────
camera.launch.py ─ /camera/camera/aligned_depth_to_color/image_raw
                             │
                             ▼
                   robot_segmenter_node        ← 🔴 없으면 전부 실패한다
                   (로봇 몸을 depth에서 지움)
                             │ /cumotion/camera_1/world_depth
                             ▼
                        nvblox_node            ← esdf_mode:=3d 필수
                             │ /nvblox_node/get_esdf_and_gradient (서비스)
                             ▼
bringup.launch.py ─▶ cumotion_planner_node     ← read_esdf_world:=True
  (실기 로봇)              │ /cumotion/move_group (액션)
                             ▼
                    move_group (cumotion:=true) ─▶ RViz 드롭다운에서 OMPL↔cuMotion
```

**실행 순서**: 호스트 카메라 → 호스트 bringup → (컨테이너) `container_setup.sh` →
segmenter → nvblox → planner → move_group. **컨테이너 셸마다 `ROS_DOMAIN_ID=93`.**

2026-08-06 결과: 계획 **5/5 성공**, `/curobo/voxels`에 점유 복셀 정상 적재.
장애물이 궤적을 실제로 바꾸는지(회피 동작)는 **아직 미검증** — 계획 성공까지만 확인했다.

### 6-0a. 🔴 `robot_segmenter_node` — 이게 없으면 로봇이 **자기 몸**을 장애물로 본다

없이 돌리면 계획이 전부 이 사유로 실패한다:

```
MotionGenStatus.INVALID_START_STATE_WORLD_COLLISION
```

nvblox는 MoveIt의 self-filter를 **안 거친다**(원본 depth를 직접 먹는다). 상세·명령은
[[ws/cobot2/context/constraints]] "nvblox 경로에는 robot_segmenter_node가 필수다"가 단일 출처.
⚠️ 세그멘터를 끼운 뒤에는 **nvblox를 재시작**해야 한다 — 기존 지도의 로봇은 안 지워진다.
⚠️ 이 노드는 `cv2`를 쓰므로 **numpy를 1.x로 내려야 뜬다**(같은 문서).

### 6-1. nvblox 실행 — 기성 example 런치 대신 `nvblox_node` 직접 실행

`nvblox_examples_bringup/realsense_example.launch.py`의 기본 리매핑(`nvblox.launch.py`의
`NvbloxCamera.realsense` 분기)은 **드라이버+`realsense_splitter_node`가 컨테이너 안에서 같이
뜨는 걸 전제**하고 `/camera0/...` 토픽을 찾는다. 이 ws는 RealSense 드라이버가 **호스트**에서
`/camera/camera/...`로 돌고 컨테이너는 `--network host`로 구독만 하는 구조라 안 맞는다 —
example 런치를 쓰지 말고 `nvblox_node`를 직접 리매핑해서 띄운다. 명령은
[[ws/cobot2/plans/2026-08-04-gpu-rental-checklist]] §7(Lightning AI 대여 GPU에서 bag 재생으로
이미 검증된 것)에서 `use_sim_time`만 빼고 그대로 가져왔다 — 그때 밟은 지뢰(§6 10개, 특히
`--params-file` 절대경로·`global_frame:=base_link`·`static_mapper.` 접두사)가 라이브 카메라에도
그대로 적용된다:

```bash
export ROS_DOMAIN_ID=93
# ⚠️ RMW는 기본값(Fast DDS) 그대로 둔다 — 2026-08-06 정정. 아래 6-2 참고
ros2 run nvblox_ros nvblox_node --ros-args \
  --params-file /workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_examples/nvblox_examples_bringup/config/nvblox/nvblox_base.yaml \
  -p global_frame:=base_link \
  -p use_lidar:=false \
  -p num_cameras:=1 \
  -p esdf_mode:=3d \
  -r camera_0/depth/image:=/camera/camera/aligned_depth_to_color/image_raw \
  -r camera_0/depth/camera_info:=/camera/camera/aligned_depth_to_color/camera_info \
  -r camera_0/color/image:=/camera/camera/color/image_raw \
  -r camera_0/color/camera_info:=/camera/camera/color/camera_info
```

### 6-1a. 🔴 `esdf_mode:=3d` — **없으면 cuMotion의 첫 요청이 nvblox를 죽인다** (2026-08-06)

`nvblox_base.yaml:33`의 기본값은 `esdf_mode: "2d"`다. 이 상태로 cuMotion이 ESDF 서비스를 부르면
nvblox가 **FATAL로 프로세스째 종료**한다:

```
[FATAL] nvblox_node: The ESDF service is only intended for mapping with 3D ESDFs.
        You're in 2D mode. To use this function set esdf_mode: 3d. Exiting.
```

**이전 판(2026-08-05)의 §6 명령이 이걸 놓친 이유**: 게이트 F는 "노드가 떴다"까지만 확인했고
**ESDF 요청을 한 번도 보내지 않았다.** 요청을 보내는 순간 처음 드러났다.

증상이 고약하다 — cuMotion 쪽에는 `Calling ESDF service` 로그만 남고 계획이 실패하는데,
**죽은 건 nvblox이지 cuMotion이 아니다.** cuMotion 로그만 보면 원인이 안 보인다.
→ **cuMotion 계획이 실패하면 `pgrep -f nvblox_node`부터 확인한다.**

2d 모드는 슬라이스 하나(높이 하나)만 만든다. `static_mapper.esdf_slice_*` 파라미터는 그
2d 슬라이스용이라 **3d에서는 의미가 없다** — 그래서 위 명령에서 뺐다.

검증(2026-08-06, cuMotion 없이 서비스를 직접 호출):

```bash
ros2 service call /nvblox_node/get_esdf_and_gradient nvblox_msgs/srv/EsdfAndGradients \
"{update_esdf: true, visualize_esdf: true, use_aabb: true, frame_id: base_link,
  aabb_min_m: {x: -1.0, y: -1.0, z: -1.0}, aabb_size_m: {x: 2.0, y: 2.0, z: 2.0}}"
```

→ **41×41×41 그리드 반환, nvblox 생존.** 관측된 곳은 실거리(0.15~0.42 m),
미관측은 `-1000.0`. `voxel_size_m`은 nvblox 자신의 **0.05**다(cuMotion에 준 `voxel_size:=0.02`가
아니다 — 그리드 해상도는 nvblox가 정한다).

**`nvblox_realsense.yaml` specialization은 얹지 않는다** — `map_clearing_frame_id: camera0_link`가
우리 TF(`camera_link`)와 안 맞는다. base yaml 기본값(`base_link`)이 맞다.

확인된 것(2026-08-06, 라이브):
- `ros2 param get /nvblox_node global_frame` → `base_link` (params-file + override 정상 반영)
- depth 15Hz 정상 수신(`ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw`,
  cyclonedds 쪽에서 측정 — Fast DDS 대역폭 문제, gpu-rental §6 지뢰#2가 라이브에서도
  재현될지 걱정했으나 이 ws는 호스트·컨테이너가 같은 머신 loopback이라 문제없었다)
- `TF lookup failed` 없음, `/nvblox_node/get_esdf_and_gradient` 서비스 정상 노출

### 6-2. ⚠️ RMW — **cyclonedds는 필요 없다. 오히려 실기를 깬다** (2026-08-06 정정)

**2026-08-06 재실측: 기본 RMW(Fast DDS)만으로 전부 된다.** 같은 컨테이너에서 nvblox를 기본
RMW로 띄우고 `ros2 service list`·`ros2 param get /nvblox_node global_frame`·
`ros2 service call .../get_esdf_and_gradient`가 전부 정상 응답했다. 아래 원래 기록은
**cyclonedds를 굳이 켰을 때** 생기는 문제를 적은 것이지, cyclonedds가 필요하다는 뜻이 아니었다.

🔴 **그리고 이 구성에서 cyclonedds를 켜면 실기가 깨진다.** `moveit.launch.py standalone:=false`가
띄우는 `dsr_moveit_controller` spawner는 **호스트**의 `/dsr01/controller_manager` **서비스**를
부르는데, 서비스는 교차 벤더가 안 된다(아래가 그 근거다). 호스트는 기본 Fast DDS이므로
**컨테이너도 기본값으로 두어야 spawner가 산다.**

→ **결론: `RMW_IMPLEMENTATION`을 설정하지 않는다.** 아래는 그래도 켜야 할 때를 위한 기록이다.

#### (원 기록) cyclonedds 노드와 대화하려면 CLI도 cyclonedds여야 한다 (2026-08-06)

`nvblox_node`를 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`로 띄운 상태에서, 그 값을 안 준 별도
셸의 `ros2 service list`/`ros2 param get`이 **120초 넘게 응답을 안 하고 멈췄다.** 반면 같은
호스트 카메라(기본 rmw_fastrtps_cpp, 호스트엔 `RMW_IMPLEMENTATION` 미설정)가 발행하는
**토픽**은 cyclonedds 쪽 `ros2 topic hz`로 문제없이 15Hz가 잡혔다 — **토픽 pub/sub 교차 벤더는
되는데, 서비스/파라미터 질의(CLI 쪽)는 CLI 자신의 RMW가 노드와 맞아야 한다.**
→ **`nvblox_node`·`cumotion_planner_node`와 대화하는 모든 셸(진단 CLI 포함)에
`export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`를 먼저 넣는다.** 컨테이너 `~/.bashrc`에
박아두면 매번 안 잊는다(gpu-rental-checklist §6 지뢰#2/#4와 같은 교훈, 그때는 대역폭이었고
이번엔 서비스 디스커버리라는 점만 다르다).

```bash
ros2 run isaac_ros_cumotion cumotion_planner_node --ros-args \
  -p robot:=m0609_rg2.xrdf \
  -p urdf_path:=/workspaces/isaac_ros-dev/m0609/m0609_kinematics.urdf \
  -p read_esdf_world:=True \
  -p esdf_service_name:=/nvblox_node/get_esdf_and_gradient \
  -p update_esdf_on_request:=True
```

nvblox는 별도로 띄운다(`isaac_ros_nvblox`, RealSense 입력).
연결은 **토픽이 아니라 서비스**다 — 계획 요청 시 pull 한다.

---

## 7. 측정 항목 (프로젝트 데이터가 되는 것)

### 7-1. ✅ OMPL vs cuMotion 계획 시간 — **1차 실측 (2026-08-06)**

조건: **로봇·카메라 없음**, `standalone:=true octomap:=false`, 세계에 장애물 0개,
관절공간 목표 `[0.5, -0.4, 1.2, 0.0, 0.9, 0.0] rad`, 시작자세 all-zeros, `plan_only`, 각 20회.
도구: `scripts/bench_planning_time.py`.

| | 1회차 | 이후 중앙값 | min | max | 성공 |
|---|---|---|---|---|---|
| OMPL — server(`planning_time`) | 26.9 ms | **16.1 ms** | 5.3 | 28.4 | 20/20 |
| OMPL — wall(액션 왕복 포함) | 1040.4 ms※ | 99.8 ms | 79.6 | 123.1 | |
| cuMotion — server | 102.1 ms | **94.7 ms** | 91.7 | 96.5 | 18/20 |
| cuMotion — wall | 190.2 ms | 199.1 ms | 192.5 | 204.9 | |

※ OMPL 1회차 wall 1초는 액션 클라이언트 첫 디스커버리다(플래너 시간 아님 — server는 26.9 ms).

**🔴 이 조건에서는 cuMotion이 OMPL보다 약 6배 느리다.** 예상과 반대 방향이다.
읽는 법을 틀리지 말 것:

- **빈 세계·관절공간 목표는 OMPL(RRTConnect)에 가장 유리한 조건이다.** 충돌 검사가 거의 없으면
  샘플링 몇 번에 끝난다. cuMotion은 장애물이 0개여도 **최적화 파이프라인 전체를 매번 돈다**
  (그래서 분산이 작다 — 91.7~96.5 ms로 ±3%. OMPL은 5.3~28.4 ms로 5배 요동).
- **따라서 이 숫자로 (a)/(b)를 결정하면 안 된다.** 결정 근거가 되는 건 **장애물이 있는 씬**
  (octomap/nvblox + CollisionObject)에서의 재측정이다 → 아래 7-2.
- 지금 이 표의 값어치는 두 가지다: ① 배선이 살아 있다는 증거 ② **빈 세계 기준선** —
  나중에 장애물을 넣었을 때 두 플래너가 각각 얼마나 느려지는지의 분모.

**미해결**: cuMotion 20회 중 **2회가 `INVALID_MOTION_PLAN(-2)`**. 플래너 노드 로그는
그 20회를 **전부 `success=True`로 반환**했으므로, 반려한 쪽은 move_group이다
(궤적 검증 단계). 재현율 10%. 원인 미특정 — 실기 전에 봐야 한다.

### 7-1a. ✅ VRAM 피크 — **2.5 GB / 8 GB. 세 노드 동시 실행 가능** (2026-08-06 실측)

`nvidia-smi --query-compute-apps=pid,used_memory --format=csv`, 실기 파이프라인 full-up 상태:

| 노드 | VRAM |
|---|---|
| `cumotion_planner_node` | 1,508 MiB |
| `robot_segmenter_node` | 660 MiB |
| `nvblox_node` | 334 MiB |
| **합계** | **약 2,500 MiB** |

- RTX 4060 Laptop **8 GB**의 31%. 셋을 동시에 띄우는 데 여유가 있다
  (계획서들이 12 GB를 가정했던 것과 무관하게 이 파이프라인은 8 GB로 충분하다)
- ⚠️ **GraspGenX는 여기 안 들어 있다.** 그건 호스트 `uv` 트랙이고 `--num_grasps 64` 기준
  별도 VRAM을 먹는다. **동시 실행은 아직 안 재봤다**
- ⚠️ **GPU는 팀 공유다.** 다른 계정이 자기 컨테이너로 같은 GPU를 쓴다 →
  [[ws/cobot2/context/constraints]] "세 계정이 동시에 로그인해..."
- 모든 노드를 내리면 `memory.used`가 **~33 MiB**로 돌아온다. 이게 "반납 완료" 판정 기준이다

### 7-2. 남은 측정

| 항목 | 방법 | 왜 |
|---|---|---|
| **OMPL vs cuMotion 계획 시간 — 장애물 있는 씬** | 7-1과 같은 스크립트, 단 `octomap:=true` + CollisionObject 배치 | **(a)/(b) 결정의 실제 근거.** 7-1은 기준선일 뿐이다 |
| 구가 로봇을 덮는가 | `/curobo/voxels` RViz 육안 | 덜 덮으면 부딪힌다 |
| octomap vs nvblox 갱신 지연 | 손을 넣었다 뺐다 | 계획서 Phase 2-1과 **같은 항목 — 중복 측정 말 것** |
| VRAM 피크 | `nvidia-smi --query-gpu=memory.used --format=csv -l 1` | 8 GB 안에서 순차 실행 가능한지 |

---

## 8. GraspGenX 실물 테스트 (컨테이너 **밖**, 별개 트랙)

컨테이너와 무관하다. 호스트에서 `uv`로 돈다.

```bash
# 1) 장면 캡처 (로봇 팔·사람을 작업공간 박스 밖으로 치우고)
cd ~/cobot2_ws
python3 scripts/capture_graspgenx_scene.py --ros-args -p scene:=00

# 2) grasp 생성
cd ~/cobot2_ws/isaac_ros-dev/src/GraspGenX
uv run python scripts/demo_scene_pc.py \
  --sample_data_dir ~/cobot2_ws/data/graspgenx_scene \
  --gripper_name onrobot_RG2 \
  --num_grasps 64
```

`--grasp_threshold` 기본 0.7. grasp가 0개면 0.3까지 내려 본다(계획서 §6 분기 E).

---
확신도: 게이트 A~F **검증됨**(컨테이너 안에서 실제로 실행). §5·§7-1은 2026-08-06에 잰 실측값이다.
단 **로봇·카메라 없이** 잰 것이고, §7-1 표는 빈 세계 기준선이지 (a)/(b) 결정 근거가 아니다.
남은 **추론**: cuMotion 2/20 `INVALID_MOTION_PLAN`의 원인, 장애물 있는 씬에서의 상대 속도.
내가 채워넣은 가정: ① 빈 세계 기준선도 기록할 값어치가 있다(장애물 씬의 분모) ② XRDF 구 6쌍
ignore는 임시 조치이고 정공법은 재피팅이다 ③ `voxel_size`는 기존 octomap과 맞춰 0.02.
확인 요청: **XRDF `link_4 ↔ rg2_base_link` 자기충돌 무시를 그대로 두고 갑니까?**
— 그리퍼가 팔뚝으로 접히는 실제 경로라 실기 모션 전에 결론이 필요하다.
