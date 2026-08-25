# K7 YG HEV CAN 안정화 계획

## 목적

`supercombo_k230`의 KIA K7 YG HEV CAN 송수신 동작을 실제 차량에서 안정적으로
사용했던 `/Users/chan/Documents/openpilot_c2`와 맞춘다. 우선 해결할 증상은 다음과
같다.

- 계기판의 LDWS 관련 아이콘이 매우 빠르게 점멸한다.
- 주행 중 간헐적으로 LKAS/MDPS 계통 경고등이 표시된다.
- 애플리케이션의 engage 상태와 Panda의 `controls_allowed`가 간헐적으로 어긋난다.
- USB 또는 프로세스 지연이 발생했을 때 CAN alive counter가 건너뛸 수 있다.

이 문서의 비교 기준은 다음 형상이다.

- `supercombo_k230`: `main`, 기준 커밋 `a0a999d`
- `openpilot_c2`: `no-big-supercombo-dlc`, 기준 커밋 `34fa405`
- `openpilot_c2`의 비교 대상 CAN 파일은 로컬 `master`와 차이가 없다.
- 차량 토폴로지: powertrain bus 0, MDPS bus 1, camera bus 2
- Panda safety: `hyundaiCommunity`, safety param 0

## 확인된 동일 동작

다음 프레임 생성 규칙은 `openpilot_c2`와 일치한다.

| 프레임 | 주소 | 송신 bus | 주기 | 비고 |
|---|---:|---:|---:|---|
| LKAS11 | `0x340` | 0, 1 | 100 Hz | K7 일반 합산 checksum |
| CLU11 | `0x4F1` | 1 | 50 Hz | active일 때 MDPS에 60 km/h 전달 |
| MDPS12 | `0x251` | 2 | 100 Hz | `ToiActive=0`, `ToiUnavail=1` |

LKAS11, CLU11, MDPS12의 signal bit 위치와 checksum 계산도
`hyundai_kia_generic.dbc` 및 `openpilot_c2` 구현과 일치한다. 따라서 버스 번호나
기본 프레임 packing을 증상의 일차 원인으로 보지 않는다.

## 확인된 차이와 위험

### 1. LDWS HUD 상태가 raw 모델 확률에 직접 연결됨

현재 `path_from_model_state()`는 좌우 lane probability를 각각 `0.3`과 즉시
비교한다. hysteresis나 유지 시간이 없어 확률이 임계값 근처에서 흔들릴 때
`CF_Lkas_LdwsSysState`가 `1`, `3`, `5`, `6` 사이를 모델 주기마다 바꿀 수 있다.

`openpilot_c2`는 K7 HUD에 전달하는 `leftLaneVisible`과 `rightLaneVisible`을 항상
`true`로 설정한다. 정상 active 상태에서는 `CF_Lkas_LdwsSysState=3`, inactive
상태에서는 `4`가 유지된다.

예상 증상:

- 모델 주기인 최대 약 20 Hz로 LDWS 차선/조향 아이콘이 점멸한다.
- 모델 lane probability 변화가 CAN HUD 상태 변화로 그대로 노출된다.

### 2. CAN RX/TX IPC가 latest-value 방식임

`K230LatestChannel`은 가장 최근 payload 하나만 보존한다. 생산자가 소비자보다
두 번 이상 먼저 publish하면 중간 batch는 복구할 수 없다.

TX batch가 유실되면 실제 CAN에서 다음 counter가 건너뛴다.

- LKAS11 `CF_Lkas_MsgCount`: 4 bit
- CLU11 `CF_Clu_AliveCnt1`: 4 bit
- MDPS12 `CF_Mdps_MsgCount2`: 8 bit

현재 `k230_pandad`는 이를 `skipped`로 기록하지만 재전송하지 않는다. 이전 실행에서
`skipped=2`가 실제로 관찰됐다.

RX도 같은 구조이므로 CLU 버튼 edge, LKAS11 seed, MDPS12 seed 또는 차량 상태
프레임이 덮어써질 수 있다. Panda firmware는 직접 수신한 버튼으로
`controls_allowed`를 변경하지만 호스트가 해당 버튼 edge를 놓치면 두 상태가
서로 달라질 수 있다.

### 3. Panda 상태가 controller 입력에 포함되지 않음

현재 `K230PandaState.controls_allowed`는 overlay에만 표시된다.
`k230_controlsd`는 Panda 상태를 구독하지 않으므로 다음 상태가 가능하다.

1. 애플리케이션은 engaged/active이다.
2. Panda는 `controls_allowed=false`이다.
3. 애플리케이션이 비영점 LKAS11을 계속 요청한다.
4. Panda safety가 LKAS11을 차단한다.

`hyundaiCommunity` tx hook은 차단 여부를 결정하기 전에 `OP_LKAS_live=20`을
갱신한다. 따라서 생성 프레임이 차단되면서 순정 camera LKAS11 forwarding도
일시적으로 억제될 수 있다. 이 경우 차량에 도달하는 유효 LKAS11 자체가 순간적으로
사라질 수 있다.

`openpilot_c2`는 enabled 상태에서 Panda `controlsAllowed`가 연속으로 일치하지
않으면 `controlsMismatch`로 제어를 해제한다.

### 4. 순정 LKAS handoff 시간이 짧음

현재는 disengage 후 `inactive_release_ms=250` 동안만 0 torque 대체 프레임을
보낸다. `openpilot_c2`의 기본 `StockLKASEnabled=1` 동작은 약 3초간 대체
프레임을 유지한 뒤 순정 LKAS로 전환한다.

Panda safety의 replacement-frame live counter는 프레임별로 감소한다. LKAS,
MDPS, CLU의 원본 주기가 다르므로 생성 프레임을 중단한 뒤 순정 프레임이 다시
forward되기 시작하는 시점도 서로 다르다. 너무 잦거나 짧은 handoff는 ECU가 보는
source와 counter의 불연속을 늘린다.

### 5. stale sendcan과 USB 오류 정책

`K230CanBatch.timestamp_ns`가 존재하지만 `k230_pandad`는 TX 전에 age를 검사하지
않는다. shared memory 또는 queue에 남은 오래된 steering 명령을 프로세스 재시작
후 전송하면 안 된다. `openpilot_c2` boardd는 1초보다 오래된 `sendcan`을 버린다.

현재 USB bulk timeout은 해당 TX batch를 유실시키고, 일부 비-timeout 오류는
Panda reconnect와 safety state 재설정을 유발할 수 있다. 단일 malformed RX
batch는 현재 버리도록 되어 있으나, 연속 오류와 물리적 disconnect는 구분해서
처리해야 한다.

## 구현 계획

### 단계 1: HUD 상태 안정화

대상:

- `src/lateral_controller.cc`
- `benchmarks/check_control_replay.cc`

작업:

- CAN LKAS HUD용 lane visibility를 모델 raw probability에서 분리한다.
- K7용 CAN HUD는 `openpilot_c2`와 같이 좌우 lane visible을 유지한다.
- active일 때 `CF_Lkas_LdwsSysState=3`, inactive일 때 `4`인지 검증한다.
- 모델 lane probability는 화면 overlay와 planner 입력에는 그대로 사용한다.
- 조향 torque, path selection, lane mode 동작은 변경하지 않는다.

완료 조건:

- lane probability를 `0.3` 위아래로 반복시켜도 CAN HUD state가 점멸하지 않는다.
- active 전환에서만 HUD state가 `3`과 `4` 사이에서 변한다.

### 단계 2: RX/TX 무손실 SPSC queue

대상:

- `src/k230_ipc.h`
- `src/common_utils.cc`
- `src/k230_pandad.cc`
- `src/k230_controlsd.cc`
- `CMakeLists.txt`
- 신규 host queue test

작업:

- 상태/모델/UI에는 기존 `K230LatestChannel`을 유지한다.
- `/k230_can`과 `/k230_sendcan`은 고정 크기 shared-memory SPSC ring queue로
  교체한다.
- producer는 queue가 가득 찼을 때 기존 데이터를 덮어쓰지 않고 오류를 기록한다.
- consumer는 한 주기에서 대기 중인 batch를 순서대로 모두 drain한다.
- controller 재시작 시 TX queue generation을 초기화해 이전 명령을 폐기한다.
- `k230_pandad` 재시작 시 남은 batch는 timestamp 검사를 통과한 경우에만 보낸다.
- queue depth, full/drop, stale-drop을 1초 로그에 표시한다.

설계 기준:

- SPSC: RX는 `pandad -> controlsd`, TX는 `controlsd -> pandad`
- 최소 500 ms 이상의 100 Hz TX burst를 저장할 수 있는 slot 수
- release/acquire ordering으로 payload publish 완료 후 write sequence 공개
- 정상 부하에서 overwrite와 sequence skip은 0

완료 조건:

- 100 Hz producer/consumer stress test에서 순서와 sequence가 보존된다.
- queue full 상황이 명시적으로 검출되고 기존 batch를 조용히 덮어쓰지 않는다.
- 실제 `pandad` 로그에서 TX/RX queue drop과 stale-drop이 0이다.

### 단계 3: Panda `controls_allowed` 동기화

대상:

- `src/k230_ipc.h`
- `src/k230_pandad.cc`
- `src/k230_controlsd.cc`
- `src/lateral_controller.*`
- controller host test

작업:

- `k230_controlsd`가 `K230PandaState`를 구독한다.
- Panda 연결, 통신 상태, TX enable, safety model/param, state freshness를
  `panda_ready`로 계산한다.
- `controls_allowed=false`일 때 torque와 `ActToi`는 즉시 0으로 만들되, engaged
  중에는 유효한 zero-frame replacement stream을 계속 보낸다.
- SET 버튼 engage 시점을 `openpilot_c2`처럼 release edge로 맞춘다.
- CANCEL은 press edge에서 즉시 disengage한다.
- 애플리케이션 engaged와 Panda controls 상태가 다를 때 원인을 로그와
  `active_block`에 남긴다.
- 브레이크 입력은 기존 요구대로 lateral engage를 해제하지 않는다.

완료 조건:

- Panda controls off 상태에서 비영점 LKAS11이 한 프레임도 생성되지 않는다.
- mismatch 중에도 checksum/counter가 유효한 0 torque LKAS11이 연속 전송된다.
- SET/CANCEL edge와 app/Panda 상태 전환이 재현 가능한 테스트로 검증된다.

### 단계 4: 순정 LKAS handoff 정합화

대상:

- `params/driving.json`
- `params/README.md`
- `src/lateral_controller.cc`
- controller host test

작업:

- `inactive_release_ms`를 `openpilot_c2` 기본 동작과 같은 3000 ms로 변경한다.
- disengage 직후 LKAS11/MDPS12/CLU11 zero stream 주기를 유지한다.
- handoff 완료 시에만 생성 프레임을 중단하고 LKAS counter seed 상태를 reset한다.
- 재-engage 시 최신 camera LKAS11 counter에서 다시 시작한다.

완료 조건:

- disengage 후 3초 동안 LKAS11/MDPS12 100 Hz, CLU11 50 Hz가 유지된다.
- handoff 전 구간의 torque와 `ActToi`는 0이다.
- 재-engage 첫 LKAS11 counter가 최신 stock seed 다음 값이다.

### 단계 5: stale/USB 보호

대상:

- `src/k230_pandad.cc`
- `src/panda_client.cc`
- codec/client host test

작업:

- TX batch age 상한을 두고 오래된 batch는 전송하지 않는다.
- timeout, partial write, retry 가능한 libusb 오류, device removal을 구분한다.
- partial write는 남은 payload를 계속 전송한다.
- 연속 malformed RX에만 reconnect를 적용하고 단일 malformed batch는 폐기한다.
- reconnect 시 RX pending batch를 비우고 TX queue의 stale batch를 폐기한다.
- safety model 설정 결과와 health의 실제 safety mode/param을 비교한다.

완료 조건:

- 오래된 비영점 steering batch가 재시작 뒤 전송되지 않는다.
- USB timeout/drop/reconnect가 별도 counter로 관찰된다.
- reconnect 뒤 safety mode가 `hyundaiCommunity:0`으로 확인되기 전에는 torque가
  활성화되지 않는다.

## 검증 계획

### 호스트 단위 테스트

- LKAS11/CLU11/MDPS12 bit packing, checksum, bus, frequency
- HUD sys state 안정성
- SET release/CANCEL press edge
- Panda controls mismatch에서 zero torque
- disengage 3초 handoff
- queue 순서, full detection, producer/consumer restart
- stale TX batch 폐기
- MDPS temporary fault debounce
- 브레이크 입력이 lateral engage를 해제하지 않음

### 로그 replay

- K7 주행 로그를 100 Hz controller 입력으로 재생한다.
- 생성 프레임 수뿐 아니라 queue에서 꺼내 최종 전달된 프레임 수를 비교한다.
- 다음 값을 기록한다.
  - 각 주소/bus별 실제 전달 주기
  - alive counter discontinuity
  - checksum 오류
  - nonzero torque while Panda controls off
  - queue full/drop/stale
  - 최대 torque와 torque rate

### 정차 차량 검증

차량을 움직이기 전에 다음 조건을 모두 만족해야 한다.

- `safety=hyundaiCommunity:0`
- `controls_allowed`와 app engaged가 의도대로 전환됨
- `pandaBlocked` 증가량 0
- `can_rx_errs`, `can_send_errs`, `can_fwd_errs` 증가량 0
- TX/RX queue full, drop, stale 0
- LKAS11 bus 0/1 100 Hz
- MDPS12 bus 2 100 Hz
- CLU11 bus 1 50 Hz
- LDWS HUD state가 active에서 3, inactive에서 4로 안정적
- CANCEL에서 torque 0, 브레이크에서는 lateral engage 유지

### 제한된 실차 검증

정차 검증을 통과한 뒤 낮은 속도와 통제된 환경에서 순차적으로 확인한다.

1. engage/disengage 반복과 계기판 아이콘
2. 직선 주행 zero/low torque
3. 운전자 토크 override
4. 완만한 곡률 추종
5. USB 부하와 모델/display 전체 파이프라인 동시 실행

경고등, `pandaBlocked`, queue drop, counter discontinuity 또는 safety mismatch가
한 번이라도 발생하면 즉시 TX를 중단하고 해당 구간 로그를 보존한다.

## 완료 판정

다음 조건을 모두 만족할 때 CAN 안정화 작업을 완료한다.

- 30분 이상의 정차 CAN 테스트에서 오류/유실/차단이 0이다.
- 제한된 실차 주행에서 LDWS 아이콘 점멸과 간헐 경고등이 재현되지 않는다.
- app engaged, Panda controls, LKAS `ActToi` 상태가 일관된다.
- openpilot_c2와 동일한 버스/주기/checksum/60 km/h MDPS spoof가 유지된다.
- 모든 변경과 검증 명령 및 결과가 이 문서 또는 후속 결과 문서에 기록된다.

## 롤백 기준

- safety mode 또는 bus topology가 예상과 다름
- Panda blocked frame 증가
- LKAS/MDPS/CLU counter 또는 checksum 오류
- 기존보다 USB reconnect 빈도 증가
- steering torque 방향, 제한 또는 rate가 달라짐

롤백 시 이 계획을 시작하기 직전의 기준 커밋으로 복귀하고 TX를 비활성화한
shadow mode에서 원인을 재현한다.

## 구현 및 검증 현황

기록 시각: 2026-07-27

### 기준점

- 구현 전 변경사항과 이 계획 문서를 `4d715c0` (`Checkpoint K7 steering and CAN
  diagnostics`)으로 커밋했다.
- 이 기준점은 원격에 push하지 않았으며, 이후 구현은 별도 커밋으로 관리한다.

### 완료된 구현

- 단계 1: CAN HUD lane visibility를 모델 확률과 분리했다. active는
  `CF_Lkas_LdwsSysState=3`, inactive는 `4`를 유지한다.
- 단계 2: `/k230_can`과 `/k230_sendcan`을 64-slot SPSC shared-memory ring
  queue로 교체했다. full 시 overwrite하지 않으며 producer 시작 시 이전 queue
  generation을 폐기한다.
- 단계 3: controller가 Panda state freshness, 연결/통신/TX 상태,
  `hyundaiCommunity:0`, heartbeat, `controls_allowed`를 확인한다. Panda가
  허용하지 않을 때 torque와 `ActToi`가 0인 replacement frame만 생성한다.
- 단계 3: SET release에서 engage하고 CANCEL press에서 disengage하도록
  `openpilot_c2`와 맞췄다. 브레이크는 lateral engage를 해제하지 않는다.
- 단계 4: `inactive_release_ms`를 3000 ms로 바꾸고 handoff 완료 뒤
  LKAS counter seed를 다시 잡도록 했다.
- 단계 5: RX/TX batch age를 100 ms로 제한하고, USB timeout/retry/partial
  write 및 연속 malformed RX를 구분했다. reconnect마다 safety mode/param을
  health에서 다시 확인한다.
- daemon 1초 로그에 queue depth/full/stale, CAN RX/TX/FWD 오류,
  Panda blocked, heartbeat, controls, USB timeout/retry, malformed RX 수를
  추가했다.

### 2026-07-27 실차 주행 후속 분석

약 3분의 실차 주행에서 제어 루프와 CAN RX는 안정적이었지만
`pandaBlocked`가 `144`에서 `363`까지 증가했다. 같은 구간에 `sendcan stale`이
초당 최대 14개 발생했다.

원인은 `k230_pandad`가 CAN RX 뒤에 읽어 둔 `now`를 TX freshness 검사에도
재사용한 경쟁 조건이었다. 그 시각 이후 `controlsd`가 새 batch를 publish하면
batch timestamp가 `now`보다 커져 실제로는 새 batch인데도 future/stale로
폐기됐다. 누락 직후 torque가 Panda의 프레임당 `+3/-7` 제한을 건너뛰면서
LKAS11이 safety에서 거부될 수 있었다.

후속 수정:

- sendcan batch를 pop한 직후 timestamp를 다시 읽어 freshness를 판정한다.
- rejected CAN echo를 주소/버스별로 1초 로그에 기록한다.
- libusb RX timeout에 부분 수신 바이트가 있으면 폐기하지 않고 decode한다.
- `pandad=-10`, `controlsd=-8`로 우선순위를 올려 모델/화면 부하와 분리한다.

차선 좌측 편향은 원본 `openpilot_c2`의 `CameraOffsetAdj=60 mm` 보정이 C++
lane planner에 이식되지 않은 차이로 확인했다. 원본과 같은 `-0.06 m` lane-line
offset을 복원했으며 사용자 path offset은 0을 유지한다. 커브 진입 반응은 torque
rate를 변경하지 않고 actuator lookahead를 `0.36 s`에서 `0.42 s`로만 조정한다.

### 완료된 자동 검증

호스트 Release 빌드 후 다음 결과를 확인했다.

```text
K230_CAN_QUEUE_OK
CONTROL_SELF_TEST_OK
check_panda_can_codec: ok
MODEL_OUTPUT_EQUIVALENCE_OK output=6012 recurrent=512 pose_offset=6000
```

검증 범위:

- queue full 비덮어쓰기, 순서, producer reopen/reset, 10,000회 push/pop
- batch freshness 경계
- HUD state 안정성
- SET/CANCEL edge, Panda gate, zero torque/`ActToi`
- 3초 handoff와 stock LKAS counter 재-seed
- 60 km/h MDPS spoof, CAN codec, model output parser 회귀

K230 Linux SDK의 RISC-V toolchain으로 `k230_pandad`,
`k230_controlsd`, `k230_overlayd`의 컴파일 및 링크도 완료했다. CAN 변경부에는
새 컴파일 경고가 없었고, overlay가 함께 빌드하면서 기존 `k230_ipc.cc`에 대한
RISC-V GCC의 `maybe-uninitialized` 경고가 다시 출력됐다.

### 남은 차량 검증

현재 저장소에는 문서가 설명하는 `K230CAN1` 주행 fixture가 없으므로 실제 로그
replay 수치 검증은 아직 실행하지 못했다. 또한 `adb devices -l`에 연결된 보드가
없어 이 단계에서는 보드/차량 TX를 실행하지 않았다. 아래 검증은 구현 완료와
별개인 실차 승인 gate로 남긴다.

- 보드 shadow mode에서 queue full/stale와 Panda 오류 counter가 0인지 확인
- 정차 TX에서 주소별 100/50 Hz, checksum, alive counter 연속성 확인
- SET/CANCEL 및 3초 stock handoff 동안 계기판 상태 확인
- 제한된 주행에서 LDWS 아이콘 점멸과 간헐 경고등 재현 여부 확인
