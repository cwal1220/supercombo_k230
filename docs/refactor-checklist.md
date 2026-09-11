# 리팩터링 체크리스트

[← 문서 목차](../README.md)

2026-09-11 코드 점검 결과다. 위험도가 낮은 단계부터 순서대로 진행하고, 각
단계를 독립 커밋으로 만든다. 작업이 모두 끝나면 이 파일은 지운다.

점검 범위: 추적 파일 164개 (src 70 / benchmarks 22 / docs 13 / tools 13),
C·C++ 약 15,000줄.

---

## 1단계 — 문서·파라미터 정합  ✅ 완료 (2026-09-11)

튜닝 문서가 실제 값과 어긋나 있다. 코드를 건드리지 않으므로 가장 먼저 한다.

### 1.1 `params/README.md` 현재값 열 수정

표에 적힌 "현재값"이 실제 JSON과 다른 항목이다. 특히 토크 게인 4개가 전부
어긋나 있어 튜닝 문서로서 위험하다.

- [x] `torque_max_lat_accel_raw` — 표 22 → 실제 **40**
- [x] `torque_kp_raw` — 표 12 → 실제 **20**
- [x] `torque_kf_raw` — 표 10 → 실제 **9**
- [x] `torque_ki_raw` — 표 1 → 실제 **3**
- [x] `steer_actuator_delay` — 표 0.46 → 실제 **0.34**
- [x] `angle_offset_deg` — 표 0.0 → 실제 **-0.7**
- [x] `path_offset_m` — 표 0.09 → 실제 **0.0**
- [x] `recording.enabled` — 표 true → 실제 **false**
- [x] 본문 서술 "`torque_max_lat_accel_raw=22`는 RK K7의 2.2 m/s^2로 환산된다"도
      실제 4.0 m/s^2에 맞춰 고친다

### 1.2 없는 파라미터를 지시하는 문장 제거

- [x] "권장 튜닝 순서" 5번의 `max_lateral_jerk`, `max_lateral_accel` — 두 키는
      JSON·파서·웹 편집기 어디에도 없다. 삭제하거나 실제 항목으로 교체
- [x] "속도별 토크 제한" 표 — 헤더와 열 정의만 있고 행이 0개
- [x] "경로 제한" 표 — 같은 상태. 바로 아래 곡률 상한 서술만 남기면 된다

### 1.3 기본값 단일 출처 정하기 — **헤더 = 배포 JSON으로 확정**

로드 실패 시 `k230_controlsd.cc:325`가 throw하므로 헤더 기본값은 파일 폴백이
아니다. 실제로 쓰이는 곳은 (a) JSON에 키가 빠졌을 때, (b) `SteeringParams`를
직접 생성하는 모든 호스트 체크·리플레이 도구다. 헤더는 이미 K7 전용 값
(`mass_kg=1816`, `wheelbase_m=2.855`, `steer_ratio=16.8`)으로 차 있었으므로
나머지 8개도 배포 JSON에 맞췄다. 이로써 `extract_lateral_dataset.cc:62`의
"실차 설정과 동일" 주석이 사실이 된다.

`steering_params.h` 멤버 초기값 / `params/steering.json` / README 표가 각각
따로 표류한다. 확인된 불일치:

| 파라미터 | 헤더 초기값 | steering.json |
|---|---:|---:|
| `torque_kp_raw` | 10 | 20 |
| `camera_offset_m` | -0.06 | 0.0 |
| `min_steer_speed_mps` | 0.3 | 1.0 |
| `steer_actuator_delay` | 0.46 | 0.34 |

- [x] 방침 결정: JSON을 단일 출처로 하고 헤더 초기값은 "파일이 없을 때의
      안전값"으로만 두는가, 아니면 헤더를 JSON과 일치시키는가
- [x] 정한 방침을 `params/README.md` 서두에 한 줄로 명시

### 1.4 사실과 어긋난 주석

- [x] `src/steering_params.h:73`, `src/steering_params.cc:57` — "steering_params.json을
      읽어"라고 하지만 실제 파일은 `params/steering.json`
- [x] `src/driving_params.*` — 같은 문제

**검증**: 키 집합 자체는 JSON·파서·README·`param_server.py` 네 곳이 이미 정확히
일치한다(누락·잉여 0건). 값만 고치면 된다.

---

## 2단계 — 죽은 코드 제거  ✅ 완료 (2026-09-11)

동작 변경 없이 지울 수 있는 것만 모았다. 제어 루프의 프레임당 힙 할당도 같이
사라진다.

### 2.1 `lateral_path` 기하 연산 (가장 큰 건)

`LateralController`가 `path`에서 읽는 것은 세 개뿐이다.

```
src/lateral_controller.cc:118   path.usable_for_steering
src/lateral_controller.cc:131   path.left_valid
src/lateral_controller.cc:132   path.right_valid
```

그런데 `path_from_model_state()`는 100 Hz로 불리면서 최대 33개 점을
`std::vector`에 push한다(`src/lateral_path.cc:51-57`). **불리언 두 개를 얻으려고
매 프레임 힙 할당을 한다.** 곡률은 이제 전부 MPC(`LateralTarget.curvatures`)에서
나온다.

- [x] `steering_curvature()` 삭제 (`lateral_path.cc:90`) — 호출처는
      `diagnostics/check_control_replay.cc:968` 뿐
- [x] `path_lateral_at()` 삭제 (`lateral_path.cc:68`) — 호출처는 같은 파일 내부와
      `check_control_replay.cc:966` 뿐
- [x] `path_curvature_at()` 삭제 (`lateral_path.cc:14`, 익명 namespace)
- [x] `LateralPathPoint` struct와 `LateralPath::points`, `LateralPath::confidence`
      삭제
- [x] `path_from_model_state()`를 점 수집 없이 도달거리·점 개수만 세도록 축소
      (판정 기준 `points.size() >= 4 && back().forward_m >= 5.0f`는 유지)
- [x] `diagnostics/check_control_replay.cc:962-968`의 해당 단언 3줄 정리

예상: `lateral_path.cc` 92줄 → 약 30줄, 100 Hz 루프 할당 제거.

### 2.2 쓰기만 하고 읽지 않는 `LateralTarget` 필드

- [x] `lookahead_x_m` — `src/openpilot_lateral_planner.cc:333`에서 쓰고 끝
- [x] `d_path_points[17]` — `openpilot_lateral_planner.cc:339`에서 쓰고 끝 (68 B)
- [x] `heading_rad` — **유지 결정**. `diagnostics/planner_replay.cc:113`이 실제로
      읽으므로 쓰기 전용이 아니다. MPC 첫 노드 psi는 플래너 회귀 분석에 쓴다

### 2.3 이름만 남은 `EffectiveSteerLimits`

`src/steering_params.cc:40-47`이 `steer_max`/`delta_up`/`delta_down`을 그대로
복사만 한다. 속도별 가변 제한이 있던 시절의 흔적이고, 그 기능은 1.2의 빈 표와
함께 이미 사라졌다.

- [x] `EffectiveSteerLimits` struct와 `effective_steer_limits()` 삭제
- [x] `hyundai_limits()`가 `SteeringParams`에서 직접 값을 채우도록 변경
- [x] 호출처 `src/lateral_controller.cc:220-221` 정리

### 2.4 잔여 실수

- [x] `src/projection.h:20-24` — `rotation_from_rpy` 선언이 `#endif` **바깥**에
      있다. 인클루드 가드 안으로 옮긴다
- [x] `src/hyundai_steering.cc:8-10` — 빈 `namespace { }` 삭제

---

## 3단계 — 빌드 통일  ✅ 대부분 완료 (2026-09-11)

Docker 없이 가는 방침으로 확정. 실사용 경로를 확인한 결과 최근(9/10) 크로스
빌드는 macOS clang + Xuantie sysroot(`build/`)였고 `build-k230-sdk/`는 존재하지도
않았다. Docker/Buildroot SDK 경로는 실질적으로 죽어 있어 제거했다.

ABI 실측: Xuantie sysroot은 glibc **2.33**으로 보드와 같고, 빌드된 6개 바이너리
전부 최대 요구 버전이 `GLIBC_2.33`이다. Buildroot SDK 없이도 ABI 보장이 유지된다.

### 3.1 빌드 디렉터리 이름 통일 — 확정

| 디렉터리 | 용도 |
|---|---|
| `build/` | macOS SDK 크로스 빌드. 업로드 기본값 |
| `build-native/` | 보드 네이티브 빌드 |
| `build-host/` | 호스트 자체 검사 (`scripts/run_host_checks.sh`) |

`build/host-checks`는 CMake 빌드 트리 안에 또 다른 CMake 빌드 트리를 넣는
구조라 `build-host`로 분리했다. 셋 다 `.gitignore`의 `/build/`+`/build-*/`에
잡힌다.

- [x] 명명 규칙 확정 후 README·build-and-deploy.md·diagnostics.md·
      verification.md·yg_panda_port.md·업로드 스크립트 전부 일치
- [x] `upload_to_board.sh` 기본값을 `build`로 변경 — 이제 macOS 빌드 산출물을
      가리킨다. 파일이 없으면 기존대로 즉시 실패한다

### 3.2 kmodel 배포 경로 이원화 해소

- [x] `upload_to_board.sh`가 `models/`에 넣도록 변경 (CMake install과 일치)
- [x] `k230_manager.py`의 두 경로 탐색 제거, `DEFAULT_KMODEL_PATH` 하나로
- [x] 보드에 남은 예전 `model/` 디렉터리는 업로드 시 안내만 한다. 원격 삭제는
      하지 않으므로 필요하면 직접 지운다

### 3.3 크로스 빌드 진입점 정리

- [x] `scripts/configure_k230_cross_build.sh`(Docker + Buildroot SDK) 제거
- [x] `docs/build-and-deploy.md`의 Docker 절 제거, glibc 경고에 Xuantie 2.33
      실측 근거 추가
- [x] 툴체인 버전 하드코딩 3곳 제거 — `configure_k230_macos.sh`가 툴체인
      디렉터리에서 GCC 버전을 자동 검출한다(`K230_XUANTIE_GCC_VERSION`으로 override)

### 3.4 호스트 체크 일괄 실행

- [x] `scripts/run_host_checks.sh` 추가 — 빌드 + 8종 실행, 실패 시 출력과 함께
      비영 종료. 저장소 루트로 `cd` 한다(부록 B)
- [x] diagnostics.md를 정본 진입점으로, verification.md·yg_panda_port.md의
      중복 절차 정리. yg_panda_port.md의 잘못된 바이너리 경로
      (`./check_control_replay`)도 수정

### 3.5 작업 트리·문서 정리 — **사용자 판단 대기**

로컬 잔여물은 전부 gitignore되어 저장소에는 영향이 없지만 1 GB를 넘는다.
삭제는 되돌릴 수 없어 손대지 않았다.

- [ ] `core` (956 MB, 코어덤프 — 사실상 쓰레기)
- [ ] `build-quant-host/`, `build-deploy-quant/` — 새 명명 규칙 밖의 옛 빌드
      디렉터리. `build-deploy-quant/`에는 로그(`final-build.log`,
      `release.json`, `stage-upload.log`)가 있어 확인 후 판단
- [ ] `gmodel_dump_dir/`, `models/quantization_work/`, `models/work/`
- [ ] `docs/can_stability_plan.md` 409줄 — 완료된 계획서. 유지 여부 결정

---

## 4단계 — 파일 병합  ✅ 완료 (2026-09-11)

`src` 추적 파일 70 → 66. 단순 이동이 아니라 의존 방향도 바로잡았다.

### 4.1 파라미터 로더 보일러플레이트 제거

- [x] `json_utils`에 `load_json_param_file(path, apply, error)` 추가 —
      파일 통째 읽기 + try/catch를 한 곳으로 모았다
- [x] 로더 3개(steering / driving / adaptive cruise)가 이제 키 목록만 쓴다
- [x] 규약 통일: adaptive cruise의 `enabled`도 `parse_json_optional_bool`로
      (기존에는 혼자 `parse_json_bool_value`를 썼다)
- [x] `calibration_service.cc`와 `k230_recordd.cc`의 JSON 읽기는 **변환하지
      않았다**. 계약이 다르다 — 전자는 try 블록 안에서 `false`를 조기 반환하고,
      후자는 키가 없으면 호출자로 throw를 전파한다. 람다로 감싸면 의미가 바뀐다

### 4.2 `steering_params` + `driving_params` → `control_params`

둘은 `LateralControllerConfig`가 함께 담고 `load_runtime_params`가 함께 읽는다.

- [x] `src/control_params.{h,cc}` 신설, 옛 4파일 제거
- [x] **의존 방향 수정**: 기존 `steering_params.h`는 `HyundaiSteeringLimits`
      때문에 CAN 헤더를 끌어왔다(파라미터 → CAN). `hyundai_limits()`를 멤버에서
      `hyundai_can.h`의 자유 함수 `hyundai_limits(const SteeringParams &)`로
      옮겨 방향을 뒤집었다. 이제 CAN 계층이 파라미터를 알고, 반대는 없다
- [x] `openpilot_torque_controller.h`가 더 이상 `vehicle_can.h`를 간접 포함하지
      않는다

### 4.3 `hyundai_steering.*` → `hyundai_can.*`

- [x] `HyundaiSteeringLimits`, `apply_hyundai_steer_torque_limits`,
      `mdps_speed_for_lkas`를 `hyundai_can.{h,cc}`로 흡수하고 2파일 제거
- [x] `CMakeLists.txt`의 `control_core` 소스 목록 갱신

---

## 5단계  ✅ 완료 (2026-09-11)

사용자 결정에 따라 진행. 5.5(HUD 분할)는 현행 유지, 5.7(clang-format)은 보류.

### 5.1 `lateral_*` 명명 — 접두사 제거로 확정

- [x] `openpilot_lateral_planner.*` → `lateral_planner.*`,
      `OpenpilotLateralPlanner` → `LateralPlanner`
- [x] `openpilot_torque_controller.*` → `torque_controller.*`,
      `OpenpilotTorqueController` → `TorqueController`
- [x] 헤더 가드, `docs/source-layout.md`, `docs/diagnostics.md` 갱신

### 5.2 `benchmarks/` → `diagnostics/`

22개 중 실제 벤치마크는 5개뿐이고 나머지는 자체 검사 9 / 검증 1 / 도구 7이었다.
문서 페이지(`docs/diagnostics.md`)와 빌드 디렉터리(`build-host`)가 이미 그렇게
부르고 있어 거기 맞췄다.

- [x] 디렉터리 개명, CMake 옵션 `SUPERCOMBO_BUILD_BENCHMARKS` →
      `SUPERCOMBO_BUILD_DIAGNOSTICS`
- [x] `diagnostics/README.md`의 옛 경로(`/tmp/supercombo_k230_verify`,
      잘못된 바이너리 경로) 수정 및 `run_host_checks.sh` 안내 추가

### 5.3 꺼져 있던 기능 3종 제거 — 앞으로 켤 계획 없음

- [x] **smooth steer**: `smooth_steer_torque()`, 상태 `steer_timer_apply_torque_`,
      상수 `kSmoothSteerRecoverStep`, 파라미터 5개
- [x] **저속 방향지시등 차단**: `update_manual_blinker_timers()`,
      `decay_manual_blinker_timers()`, `manual_blinker_block_reason()`,
      상태 `lanechange_manual_timer_`, 파라미터 `turn_steering_disable`,
      `manual_steer_disable_frames`
- [x] **`no_smart_mdps`**: gate와 HUD 라벨 `no_smart_mdps_low_speed`
- [x] `params/*.json`, `scripts/param_server.py`, `params/README.md` 동기화

### 5.4 중복 보정 파라미터 최소화

- [x] `camera_offset_m` 제거 — `path_offset_m`만 남긴다. 둘 다 0.0이었고
      README도 중복임을 인정하고 있었다. `LanePlanner`가 차선에 더하던 오프셋이
      사라져 생성자도 단순해졌다
- [x] `roll_rad`(정적) 제거 — `live_bank_compensation`이 ESP12 실측으로 같은
      편경사를 실시간 추정한다. 값이 0이어서 동작 변화는 없다.
      `torque_lat_accel_offset`(센서에 안 보이는 기계 편향)은 목적이 달라 유지
- [x] 따라 죽은 `kGravity` 상수 2개 제거

### 5.5 `overlay_renderer.cc` 분할 — **현행 유지 결정**

### 5.6 `AIBase` 제거

- [x] `src/ai_base.{h,cc}` 제거, nncase 인터프리터를 `SupercomboModel`이 직접
      소유한다. 헤더의 `using namespace nncase::runtime` / `using std::string` /
      `using std::vector` 오염이 사라졌다
- [x] 아무도 읽지 않던 `each_input_size_by_byte_`/`each_output_size_by_byte_`와
      그것만 채우던 datatype 분기 약 50줄을 함께 제거

### 5.7 코드 스타일 — **보류**

- [ ] `.clang-format` 도입은 나중에. 전 파일 diff가 발생한다
- [ ] 인클루드 가드 혼용(`#pragma once` / `#ifndef`)
- [ ] `include/mmz.h` + `src/mmz.c` 경계 불일치

---

## 부록 — 작업 중 발견한 별건  ✅ 완료 (2026-09-11)

### A. 리플레이 참조식이 구현과 어긋나 있었다 — 수정

`check_control_replay`의 픽스처 리플레이 경로(`.k230can` 인자를 줄 때만 도는
경로)가 `max_curvature_error < 1e-6f`를 단언하는데, 참조식이 썩어 있었다.

**재현**: 합성 K230CAN1 픽스처(61초, 73,200 레코드, 0.5~90 kph 스윕)를 만들어
실패를 확인했다. 원인은 두 가지였다.

1. **상수 복제**: 참조식이 `max_lateral_accel = 3.0`, `max_curvature = 0.2`를
   하드코딩했으나 구현은 `3.3`, `0.3`이다. 속도 하한도 `0.1` vs `1.0`으로
   달랐고, `model_t`를 float으로 재계산해 `model_t_idx`의 double 계산과
   미세하게 어긋났다.
2. **입력 자체가 달랐다**: 테스트가 참조식에 `cluster_speed_raw`를 먹였는데
   구현은 휠 속도 평균(`control_speed_kph`)으로 곡률을 낸다. 두 식이 애초에
   다른 속도를 보고 있었으므로 상수를 맞춰도 일치할 수 없었다.

**수정**:

- [x] 한계 상수 5개를 `lateral_controller.cc` 익명 namespace에서
      `lateral_controller.h`로 올려 구현과 참조가 **같은 값을 공유**하게 했다.
      숫자 복제가 사라져 다시 어긋날 수 없다
- [x] 참조식이 `model_t_idx()`를 그대로 쓰도록 바꿔 정밀도 차이 제거
- [x] 테스트가 `result.control_speed_kph`를 참조식에 넘기도록 수정
- [x] 함수명 `original_lag_adjusted_curvature` → `reference_lag_adjusted_curvature`
      ("original"은 이제 openpilot 원본이 아니라 현 구현의 전사본이다)

**검증**: 합성 픽스처에서 `REPLAY_OK ... curvature_err=0.00000000`,
active 6101/6101 틱. 기본 self-test도 통과.

> 픽스처는 `/tmp`에 만든 합성 데이터라 저장소에 넣지 않았다. 실주행
> `.k230can` 픽스처로도 한 번 돌려보면 좋다.

### B. 호스트 체크는 저장소 루트에서 실행해야 한다

`check_adaptive_cruise`는 `params/`를 상대경로로 읽는다.

- [x] `run_host_checks.sh`가 저장소 루트로 `cd` 하도록 작성

---

## 남은 항목

- [ ] **로컬 잔여물 정리** (사용자 판단) — `core`(956 MB),
      `build-quant-host/`, `build-deploy-quant/`(로그 3개 포함),
      `gmodel_dump_dir/`, `models/quantization_work/`, `models/work/`.
      전부 gitignore되어 저장소에는 영향이 없지만 1 GB를 넘는다. 삭제는
      되돌릴 수 없어 손대지 않았다
- [ ] **`.clang-format` 도입** (5.7) — 전 파일 diff가 발생하므로 별도 커밋으로
