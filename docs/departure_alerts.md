# 정차 출발 알림

## 동작

`k230_controlsd`가 정차 중 두 종류의 이벤트를 판단한다.

- 선행 차량 출발: 모델의 vision lead 거리가 기준보다 0.5 m 이상 증가하고
  상대속도가 0.5 m/s를 넘는 상태가 0.3초 유지되면 알린다.
- 신호 변경 추정: 선행 차량이 없는데도 모델이 여기서 멈추겠다는 짧은 경로를
  예측한 채 3초 넘게 정차하면 신호 대기로 보고 적색 신호등을 표시한다. 이후
  경로가 25 m 이상 열리면 신호 변경을 알린다. 앞차가 있으면 그건 정체이므로
  신호 대기로 무장하지 않고, 선행 차량 출발 알림이 그대로 담당한다.

한 번 정차할 때 하나의 이벤트만 발생한다. 기어가 `D`가 아니거나 가속 페달을
누르거나 차량이 다시 움직이면 다음 정차 주기를 준비한다. 알림은 engage 여부와
무관하며 LCD에 3초 동안 표시되고, `k230_overlayd`가 보드의 passive piezo에
짧은 PWM 시퀀스를 한 번 재생한다. 선행 차량 출발과 신호 변경은
`signal_changed` 시퀀스를 공유한다.

피에조 알람은 다음 상태 전이에 적용된다.

- `signal_changed`: 선행 차량 출발 또는 신호 변경
- `engage`: 제어 engage 성공
- `disengage`: 제어 disengage 또는 fault에 의한 해제
- `activated`/`deactivated`: engage 상태는 유지되지만 조향 active gate가 바뀐 경우
- `unavailable`: 제어/Panda 상태가 stale이 되거나 Panda/조향 fault가 검출됨
- `unable`: engage 조건을 만족하지 못한 상태에서 engage 명령을 거부할 때

`engage`는 안정적인 상승 다중음, `disengage`는 하강 다중음으로 재생한다.
둘 다 보드 Python 알람과 같은 고정 50% PWM 시퀀스를 공유하므로, PWM duty를
짧게 반복 변경할 때 생기는 클릭/찌그러짐이 없다. `activated`/`deactivated`는
engage 상태를 유지한 active gate 전이에만 사용한다.

실제 차량/제어 조건으로 engage가 거부되면 LCD 하단에
`UNABLE TO ENGAGE: <사유>`를 3초간 표시하고, openpilot의 refuse 알림에
대응하는 `unable` 피에조 시퀀스를 재생한다. Panda의 `not ready`/`controls off`는
SET edge와 health 응답 사이의 정상적인 비동기 구간이므로 최대 1초 동안 대기한다.
그 사이 Panda 허가가 들어오면 `engage`만 한 번 알리고, 잠깐 뒤의 active 전이에는
중복 `activated` 차임을 붙이지 않는다. 허가가 끝내 오지 않거나 Panda 회복 뒤
다른 정적 조건이 남아 있으면 그때 `UNABLE`로 거부한다. 이미 engage된 상태에서
active gate가 나중에 풀리는 전이만 `activated`로 알린다.

주행 중 Panda health 스냅샷이 한 번 비거나 지연되는 경우에는 직전 정상
스냅샷을 최대 100 ms만 유지해 LKAS active 아이콘이 한 프레임 깜박이지 않게
한다. transport와 safety가 유효한 상태의 `controls_allowed=0`은 이 유예를
사용하지 않고 즉시 제어를 차단한다. heartbeat/연결/safety 스냅샷 오류는
최대 100 ms 뒤에도 계속되면 제어를 차단한다.

환경 변수:

- `K230_PIEZO_BUZZER=0`: PWM 피에조 알람 비활성화
- `K230_PIEZO_PIN=46|47`: PWM 피에조 핀 선택. 기본값은 `46`

IOMUX 설정에는 보드의 `devmem` 명령(`/sbin/devmem`)과 `/dev/mem` 접근이
필요하다. PWM/IOMUX를 사용할 수 없어도 주행 파이프라인은 계속 실행되고 LCD
알림은 유지된다.

## 판단 근거

신호 대기 판별에 정지선 출력을 쓰지 않는다. 배포 모델(openpilot v0.9.4)이
`stop_lines`를 내지 않기 때문이며, 대신 "앞차 없음 + 모델이 정지 계획"을 같은
근거로 삼는다. 알림 발동 조건은 정지선과 무관하게 모델 경로가 25 m 이상
열리는 것이라, 무장이 헛나가도 길이 실제로 열리지 않는 한 알림은 뜨지 않는다.

현재 차량의 레이더 입력은 사용하지 않는다. 모델의 lead x에서 카메라-레이더 기준 거리 1.52 m를 빼고, 모델 lead 속도에서 자차 속도를
빼 상대속도를 계산한다. lead 확률은 50% 이상이어야 하며, vision 거리 노이즈로 인한
오경보를 막기 위해 거리와 상대속도 조건을 0.3초 함께 확인한다.

## 계기판 차임 실차 검증

현재 형상은 계기판 차임 CAN을 송신하지 않는다.

- C2의 `chimeAtResume`은 차량 계기판이 아니라 장치의 `soundd`가
  `dingdong.wav`를 재생한다.
- K7의 `LKAS11`(`0x340`)에 있는 `CF_Lkas_SysWarning`은 독립 차임이 아니라
  조향/LDWS 경고 상태다. 차임 용도로 변경하면 아이콘 점멸이나 경고등을 만들 수
  있다.

2026-07-30에 K7 YG HEV 실차에서 다음 조건을 확인했다.

- 순정 `LFAHDA_MFC`(`0x485`)는 카메라 bus 2에서 `03 00 00 00`으로 약 20 Hz
  수신된다.
- 순정 포워딩을 억제한 뒤 `HDA_Chime`을 300 ms 동안 단독 송신했다.
- 그랜저 IG에서 쓰는 `LKAS11.CF_Lkas_SysWarning=9`를 송신했다.
- C2의 K7 YG 경로와 같은 `CF_Lkas_SysWarning=3`,
  `CF_Lkas_LdwsSysState=3`을 2초 동안 송신했다.
- 마지막 시험은 기어 `D`, 클러스터 속도와 네 바퀴 속도 모두 `0 km/h`에서
  수행했다.
- 모든 시험에서 Panda 차단, CAN 송신 오류, checksum 오류는 없었지만 계기판
  표시와 차임 모두 반응하지 않았다.

따라서 K7 YG HEV에서는 이 두 CAN 경로를 계기판 차임으로 사용하지 않는다.
정차 출발 알림은 보드 PWM 피에조와 LCD 표시만 사용한다.
