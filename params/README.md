# K7 YG HEV 파라미터 안내

이 디렉터리에는 KIA K7 YG HEV용 조향 제어, 주행 제한, 비전 크루즈 및
카메라 캘리브레이션 값이 들어 있다. `k230_k7_controlsd`는 세 런타임 JSON
변경을 100ms 이내에 감지하고 다음 제어 주기에 즉시 반영한다. 제어 상태와
PID 상태는 유지되며 engage 여부에 따른 적용 지연은 없다.

기본 경로는 실행 디렉터리의 `params/`이다. 다른 디렉터리를 사용하려면
`K230_PARAMS_DIR=/path/to/params`를 지정한다.

## 튜닝 전 주의사항

- 조향 토크, 차량 고정값과 CAN 통신값은 반드시 정차 상태에서 수정한다.
- 비전 크루즈 주행 튜닝은 운전자가 직접 조작하지 말고 동승자가 한 항목씩 소폭 변경한다.
- Panda는 먼저 `nooutput` 또는 TX 비활성 상태에서 데이터와 부하를 검증한다.
- 한 번에 하나의 값만 조금씩 바꾸고, 변경 전후 로그를 남긴다.
- 허용 범위를 벗어난 숫자는 로더가 아래 표의 범위로 제한한다.
- JSON은 주석을 지원하지 않으므로 설명은 이 문서에서 관리한다.
- `mdps_speed_spoof_kph`, 토크 부호, 차량 제원은 다른 차량 값으로 임의 변경하지 않는다.
- 웹 편집기는 `K230_ENABLE_PARAM_SERVER=1`일 때 기본 8080 포트에서 실행된다.

## k7_yg_driving.json

### 상태와 CAN

| 파라미터 | 현재값 | 단위 / 허용 범위 | 설명 |
|---|---:|---|---|
| `model_timeout_ms` | 250 | ms / 50~2000 | 모델 경로가 마지막으로 갱신된 뒤 유효하다고 인정하는 시간이다. 초과하면 조향 경로를 사용할 수 없다. |
| `vehicle_state_timeout_ms` | 500 | ms / 50~2000 | 차량 CAN 상태와 yaw-rate 신호의 freshness 제한이다. 초과하면 제어를 차단한다. |
| `inactive_release_ms` | 3000 | ms / 0~5000 | disengage 직후 순정 LKAS로 넘기기 전에 0 토크 해제 프레임을 유지하는 시간이다. openpilot_c2의 기본 handoff 시간과 같다. |
| `mdps_speed_spoof_kph` | 60.0 | km/h / 30~100 | 저속에서도 MDPS가 LKAS 조향을 허용하도록 MDPS 버스의 CLU11 속도를 위조하는 값이다. K7 YG HEV 기준값은 60 km/h다. |

### 운전자 조향 보호

| 파라미터 | 현재값 | 단위 / 허용 범위 | 설명 |
|---|---:|---|---|
| `lane_change_min_speed_kph` | 30.0 | km/h / 0~80 | 이 속도 미만에서 운전자 토크 보호를 강화하는 기준이다. `turn_steering_disable=true`일 때는 저속 방향지시등 조향 차단 기준으로도 사용한다. |
| `manual_steer_disable_frames` | 50 | frame / 0~500 | 저속에서 한쪽 방향지시등을 켰을 때 자동 조향을 차단할 프레임 수다. 제어 주기가 100Hz이므로 50 frame은 약 0.5초다. 현재 `turn_steering_disable=false`라 비활성이다. |
| `driver_torque_threshold` | 170 | MDPS raw torque / 0~500 | 이 값보다 큰 운전자 조향 토크가 저속에서 감지되면 요청 토크를 점진적으로 줄인다. Nm 단위가 아니다. |

### 경로 제한

| 파라미터 | 현재값 | 단위 / 허용 범위 | 설명 |
|---|---:|---|---|
| `max_lateral_jerk` | 5.0 | m/s^3 / 0.1~20 | 목표 곡률의 프레임 간 변화량을 속도에 맞춰 제한한다. 작게 설정하면 조향 변화가 부드러워지지만 추종이 느려질 수 있다. |
| `max_lateral_accel` | 3.3 | m/s^2 / 0.5~5 | 속도별 최대 목표 곡률을 횡가속도 기준으로 제한한다. |

최종 목표 곡률의 절대 상한은 K7 고정값 `0.3 1/m`로 적용되며 설정 항목으로 노출하지 않는다.

## k7_yg_adaptive_cruise.json

이 기능은 비전 모델의 선행차 거리와 상대속도를 이용해 순정 고정형 크루즈의
`SET-`와 `RES+` 버튼만 대신 누른다. 스로틀이나 브레이크를 직접 제어하지 않으므로
차간거리 유지와 감속을 보장하지 않으며 운전자가 항상 제동을 담당해야 한다.

| 파라미터 | 현재값 | 단위 / 허용 범위 | 설명 |
|---|---:|---|---|
| `enabled` | true | bool | 비전 기반 순정 크루즈 버튼 조절을 켠다. `false`로 바꾸면 진행 중인 자동 버튼 펄스와 세션을 즉시 해제한다. |
| `lead_probability_threshold` | 0.5 | ratio / 0.2~0.99 | 속도 제어에 사용할 모델 선행차의 최소 확률이다. |
| `standstill_gap_m` | 5.0 | m / 2~20 | 속도와 관계없이 목표 차간거리에 더하는 기본 거리다. |
| `following_time_s` | 1.8 | s / 0.8~4.0 | 현재 속도에 곱해 동적 차간거리를 만드는 시간 간격이다. |
| `gap_correction_gain` | 0.25 | gain / 0.05~1.0 | 실제 거리와 목표 거리의 오차를 목표 속도 보정으로 바꾸는 비율이다. |
| `max_slowdown_correction_mps` | 4.0 | m/s / 0.5~10 | 선행차가 가깝거나 느릴 때 목표 속도를 낮출 수 있는 최대 보정량이다. |
| `max_speedup_correction_mps` | 2.5 | m/s / 0~5 | 차간거리가 충분할 때 선행차 속도보다 높게 잡을 수 있는 최대 보정량이다. 최초 SET 상한은 넘지 않는다. |
| `deceleration_rate_kph_per_s` | 1.5 | km/h/s / 0.5~5 | `SET-` 이후 실측한 차량 감속 응답이다. 2 km/h 설정 단계를 이 값으로 나눈 시간을 연속 감속 명령의 최소 응답 시간으로 사용하고, 그동안 가까워질 선행차 거리도 미리 반영한다. |
| `lead_hold_s` | 0.6 | s / 0.1~2.0 | 비전 검출이 잠시 끊겨도 마지막 선행차를 유지하는 시간이다. |
| `lead_restore_delay_s` | 2.0 | s / 1~10 | 선행차가 사라진 뒤 최초 SET 상한으로 복귀하거나, 마지막 `SET-` 뒤 `RES+`로 명령 방향을 바꾸기 전 대기 시간이다. |
| `command_interval_s` | 1.0 | s / 0.5~5.0 | 연속 버튼 펄스를 시작할 수 있는 최소 간격이다. |
| `button_pulse_frames` | 5 | 100Hz frame / 1~10 | 한 번의 버튼 조작을 차량에 전달하는 연속 CLU11 프레임 수다. K7 검증 전에는 기본값을 유지한다. |

## k7_yg_steering.json

### 기본 토크 제한

| 파라미터 | 현재값 | 단위 / 허용 범위 | 설명 |
|---|---:|---|---|
| `enabled` | true | bool | `false`면 조향 컨트롤러를 비활성화한다. |
| `steer_max` | 384 | CAN torque / 0~384 | 가변 제한 또는 fault 회피 모드에서 사용할 최대 요청 토크이자 전체 상한이다. |
| `steer_delta_up` | 3 | CAN torque/frame / 0~20 | 가변 제한 또는 fault 회피 모드의 프레임당 토크 증가 한도다. |
| `steer_delta_down` | 7 | CAN torque/frame / 0~30 | 가변 제한 또는 fault 회피 모드의 프레임당 토크 감소 한도다. |
| `steer_driver_allowance` | 50 | MDPS raw torque / 0~300 | 운전자 토크 제한 계산에서 허용하는 기본 여유값이다. |
| `steer_driver_multiplier` | 2 | 배수 / 0~10 | 운전자 토크가 최종 허용 토크에 미치는 배율이다. |
| `steer_driver_factor` | 1 | 배수 / 0~5 | MDPS 운전자 토크 입력에 적용하는 계수다. |
| `steering_pressed_threshold` | 150 | MDPS raw torque / 0~500 | 토크 PID의 적분을 멈추는 운전자 조향 감지 기준이다. RK openpilot과 같이 5프레임 필터를 거치며, CAN 안전 제한용 `steer_driver_allowance`와는 별개다. |

### 속도별 토크 제한

| 파라미터 | 현재값 | 단위 / 허용 범위 | 설명 |
|---|---:|---|---|
| `variable_steer_max` | false | bool | `true`면 30~100 km/h 구간에서 `steer_max`에서 `steer_max_base`로 보간한다. 현재는 base 값을 고정 사용한다. |
| `variable_steer_delta` | false | bool | `true`면 속도에 따라 delta 값을 primary 값에서 base 값으로 보간한다. |
| `steer_max_base` | 384 | CAN torque / 0~384 | 일반 주행에서 실제 사용하는 최대 토크다. |
| `steer_delta_up_base` | 3 | CAN torque/frame / 0~20 | 일반 주행의 프레임당 토크 증가 한도다. 100Hz 기준으로 적용된다. |
| `steer_delta_down_base` | 7 | CAN torque/frame / 0~30 | 일반 주행의 프레임당 토크 감소 한도다. 100Hz 기준으로 적용된다. |

### OpenPilot 토크 컨트롤러

`*_raw` 값은 OpenPilot 파라미터 표현을 유지한다. 현재
`torque_max_lat_accel_raw=22`는 RK K7의 2.2 m/s^2로 환산된다. `kp`, `kf`, `ki`는
각각 `raw * 0.1 / max_lat_accel`로 변환되고 friction은 `raw * 0.001`로
변환된다.

| 파라미터 | 현재값 | 단위 / 허용 범위 | 설명 |
|---|---:|---|---|
| `torque_max_lat_accel_raw` | 22 | 0.1 m/s^2 / 1~80 | 토크 컨트롤러가 정규화에 사용하는 최대 횡가속도다. RK K7의 `latAccelFactor=2.2`에 맞춘 값이다. |
| `torque_kp_raw` | 12 | raw / 0~100 | 횡가속도 오차의 비례 이득이다. |
| `torque_kf_raw` | 10 | raw / 0~100 | 목표 횡가속도 feed-forward 이득이다. RK처럼 횡가속도 1.0을 그대로 feed-forward한 뒤 2.2로 정규화한다. |
| `torque_ki_raw` | 1 | raw / 0~100 | 횡가속도 오차의 적분 이득이다. |
| `torque_friction_raw` | 100 | 0.001 m/s^2 / 0~300 | 조향계 마찰을 넘기 위한 feed-forward 보상값이다. RK K7과 같은 0.1이다. |
| `torque_use_angle` | true | bool | `true`면 조향각 기반 실제 곡률을 사용한다. `false`면 유효한 ESP yaw-rate가 필요하며 두 값을 속도에 따라 혼합한다. |
| `torque_angle_deadzone_raw` | 0 | 0.1 degree / 0~50 | 조향각 기반 오차 deadzone이다. RK K7 기본값과 같이 비활성화했다. `torque_use_angle=true`일 때 적용된다. |
| `torque_output_sign` | -1 | 부호 / -1 또는 1 | 토크 출력 방향이다. K7 YG HEV에서는 -1을 사용한다. 잘못 바꾸면 반대 방향으로 조향할 수 있다. |

### Smooth steer

`smooth_steer_method=1`일 때만 나머지 smooth 파라미터가 적용된다. wait 값은
초가 아니라 100Hz 제어 주기마다 토크 배율에서 차감하는 비율이다.

| 파라미터 | 현재값 | 단위 / 허용 범위 | 설명 |
|---|---:|---|---|
| `smooth_steer_method` | 0 | mode / 0~1 | 0은 일반 운전자 토크 fade, 1은 조향각과 운전자 입력에 따른 smooth steer를 사용한다. |
| `smooth_max_steering_angle_deg` | 90.0 | degree / 0~180 | smooth 모드에서 토크를 줄이기 시작하는 절대 조향각이다. |
| `smooth_max_driver_angle_wait` | 0.002 | ratio/frame / 0~1 | 큰 조향각에서 운전자 조향까지 감지될 때 프레임마다 줄일 토크 배율이다. |
| `smooth_max_steer_angle_wait` | 0.001 | ratio/frame / 0~1 | 큰 조향각에서 운전자 입력이 없을 때 프레임마다 줄일 토크 배율이다. |
| `smooth_driver_angle_wait` | 0.001 | ratio/frame / 0~1 | 일반 조향각에서 운전자 조향이 감지될 때 프레임마다 줄일 토크 배율이다. |

### 차량 모델과 지연

| 파라미터 | 현재값 | 단위 / 허용 범위 | 설명 |
|---|---:|---|---|
| `steer_ratio` | 16.8 | ratio / 8~25 | 핸들 조향각과 전륜 조향각의 비율이다. RK의 KIA K7 HEV 차량값과 동일하다. |
| `tire_stiffness_factor` | 1.0 | 배율 / 0.2~2.0 | 기준 타이어 횡강성에 적용하는 차량별 보정 계수다. RK K7의 기본 배율과 동일하다. |
| `steer_actuator_delay` | 0.46 | second / 0.01~1.0 | 현재 시점의 목표 곡률을 계산할 때 보상하는 조향 액추에이터 지연이다. 값을 키우면 MPC 경로를 조금 더 앞에서 읽어 커브 진입을 선행하지만, 과도하면 조향이 빨라지거나 오버슈트할 수 있다. |
| `angle_offset_deg` | 0.0 | degree / -10~10 | 조향각 센서의 직진 오프셋이다. 실제 곡률 추정 전에 센서 각도에서 뺀다. |
| `roll_rad` | 0.0 | radian / -0.2~0.2 | 도로 또는 차량 roll에 의한 횡가속도와 곡률 보정값이다. |
| `mass_kg` | 1816.0 | kg / 1000~2600 | 차량 모델과 타이어 횡강성 계산에 사용하는 차량 질량이다. |
| `wheelbase_m` | 2.855 | m / 2.0~3.5 | 차량 축거다. |
| `center_to_front_ratio` | 0.4 | wheelbase ratio / 0.2~0.7 | 무게중심에서 전축까지 거리의 축거 대비 비율이다. |
| `steer_ratio_rear` | 0.0 | ratio / -0.5~0.5 | 후륜 조향 보정 계수다. K7은 후륜 조향이 없으므로 0을 사용한다. |
| `camera_offset_m` | 0.0 | m / -1~1 | 차량 중심에 대한 카메라 위치를 lane line에 보정한다. 양수는 목표 차선 중심을 우측, 음수는 좌측으로 이동한다. 현재는 최종 경로 오프셋만 사용하므로 0을 유지한다. |
| `path_offset_m` | 0.09 | m / -1~1 | 차선/랜리스 선택이 끝난 최종 주행 경로 전체에 적용하는 사용자 횡방향 보정이다. 양수는 목표 주행 위치를 우측, 음수는 좌측으로 이동한다. 현재 K7 실차 기준값은 `+0.09 m`다. |
| `invert_steer` | false | bool | 목표 곡률 부호를 반전한다. K7에서는 사용하지 않는다. |
| `min_steer_speed_mps` | 0.3 | m/s / 0~5 | 이 속도 미만에서는 토크 컨트롤러 출력을 0으로 만든다. |

### 조향각 및 LKAS fault 보호

| 파라미터 | 현재값 | 단위 / 허용 범위 | 설명 |
|---|---:|---|---|
| `max_steering_angle_deg` | 90.0 | degree / 0~360 | fault 회피 모드가 꺼졌을 때 자동 조향을 허용할 절대 조향각 한도다. 0이면 gate를 사용하지 않고, 0보다 크고 90 이하이면 설정값을 그대로 적용한다. 90보다 크면 정지 시 설정값+60도에서 20 km/h의 설정값까지 선형으로 줄어든다. |
| `avoid_lkas_fault_enabled` | true | bool | 큰 조향각이 지속될 때 토크는 유지하고 steer request만 잠시 끊는 RK openpilot 방식의 fault 회피 로직을 사용한다. |
| `avoid_lkas_fault_max_angle_deg` | 85.0 | degree / 1~180 | fault 회피 카운터를 증가시키는 절대 조향각 기준이다. |
| `avoid_lkas_fault_max_frames` | 89 | frame / 0~300 | 85도 이상 조향각이 지속될 때 허용하는 프레임 수다. 이후 2프레임 동안 request를 끊고 다시 허용한다. |
| `avoid_lkas_fault_beyond` | false | bool | fault 회피가 켜진 저속·큰 조향각 조건에서 primary 토크 제한을 사용하는 확장 옵션이다. |
| `no_smart_mdps` | false | bool | `true`면 `min_steer_speed_mps` 미만에서 제어 자체를 차단한다. |
| `turn_steering_disable` | false | bool | `true`면 `lane_change_min_speed_kph` 미만에서 한쪽 방향지시등을 켰을 때 지정 프레임 동안 조향을 차단한다. |
| `ldws_car_fix` | false | bool | LKAS11 생성 시 차량별 LDWS 보정 플래그를 적용한다. |

## calibration.json

이 파일은 일반 튜닝 파일이 아니라 온라인 카메라 캘리브레이션의 저장 상태다.
`SUPERCOMBO_CALIB_AUTO=1`일 때 모델 pose와 실제 차량 속도로 갱신된다. 보드나
카메라 장착 각도가 달라지면 기존 값을 그대로 사용하지 말고 다시 캘리브레이션한다.

| 파라미터 | 현재값 | 단위 | 설명 |
|---|---:|---|---|
| `version` | 1 | schema version | 저장 형식 버전이다. 사용자가 변경하지 않는다. |
| `rpy_rad` | `[-0.000135, 0.036573, 0.020478]` | radian `[roll, pitch, yaw]` | 카메라 자세 보정값이다. 현재 약 `[-0.008, 2.095, 1.173]`도다. |
| `spread_rad` | `[0.000047, 0.003080, 0.001292]` | radian `[roll, pitch, yaw]` | 유효 블록 사이 캘리브레이션 값의 분산 범위다. 작을수록 관측이 안정적이다. |
| `valid_blocks` | 18 | block | 저장된 유효 캘리브레이션 블록 수다. 5개 이상이면 calibrated 상태가 될 수 있다. |

캘리브레이션을 처음부터 다시 진행하려면 정차 상태에서 애플리케이션을 종료한 뒤
보드의 `params/calibration.json`을 별도로 백업하고 삭제한 다음 파이프라인을 시작한다.
실차에서는 충분한 속도로 직선에 가까운 도로를 주행하면서 카메라가 움직이지 않게
고정해야 한다.

## 권장 튜닝 순서

1. 카메라 장착을 고정하고 온라인 캘리브레이션을 완료한다.
2. `camera_offset_m`, `path_offset_m`로 차선 중심 위치를 먼저 맞춘다.
3. `angle_offset_deg`, `steer_ratio`, 차량 제원이 실제 차량과 맞는지 확인한다.
4. `torque_kp_raw`, `torque_ki_raw`, `torque_kf_raw`, `torque_friction_raw`을 한 항목씩 조정한다.
5. 토크가 정상적으로 추종된 뒤 `steer_actuator_delay`, `max_lateral_jerk`, `max_lateral_accel`을 조정한다.
6. 마지막으로 운전자 토크 보호와 fault 회피 옵션을 검증한다.

각 단계에서 disengage 가능 여부, 운전자 개입 시 즉시 토크가 줄어드는지, CAN 오류와
MDPS fault가 없는지를 먼저 확인한다.
