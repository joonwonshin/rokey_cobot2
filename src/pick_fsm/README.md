# pick_fsm

> 🚨 **이 런치를 띄우면 실기가 실제로 움직인다.** `dry_run`(계획만) 파라미터는
> **2026-08-09 제거**했다 — 팔만 막고 그리퍼는 최대 힘으로 실제 개폐되던 반쪽 안전이라
> 오히려 위험했다. 남은 소프트 게이트는 `require_approval:=true`(기본, `/pick/approve`)
> 하나이고 **최종 안전장치는 물리 비상정지 버튼이다.**
> 🔴 옛 명령줄의 `dry_run:=true` 를 붙여도 **경고 없이 무시된다** — 안 막아준다.

레퍼런스(상태머신·인터페이스·파라미터·검증 상태·`robot_safety_node`·rqt 패널)는
**[`src/PACKAGES.md`](../PACKAGES.md#pick_fsm)**로 옮겼다.
실행 절차·기능확인은 워크스페이스 루트 **[`README.md`](../../README.md)**.
플래너 파이프라인(OMPL/cuMotion) 등록·선택은 루트 README **7-1절**.
