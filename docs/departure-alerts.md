# 정차 출발 알림

[← Documentation index](../README.md)

## 동작

`k230_controlsd`가 정차 중 두 종류의 이벤트를 판단한다.

- 선행 차량 출발: 모델의 vision lead 거리가 기준보다 0.5 m 이상 증가하고
  상대속도가 0.5 m/s를 넘는 상태가 0.3초 유지되면 알린다.
- 신호 변경 추정: 모델이 여기서 멈추겠다는 짧은 경로(5 m 미만)를 예측한 채
  3초 넘게 정차하면 신호 대기로 보고 적색 신호등을 표시한다. 이후 경로가 10 m
  넘게 0.3초 열리면 신호 변경을 알린다. 앞차가 있어도 무장한다. v0.9.4의 lead
  확률은 앞차 유무를 가르지 못하고, 앞차가 출발하면 경로도 같은 식으로 열리기
  때문이다. 같은 프레임에 선행 차량 출발도 잡히면 그쪽 알림이 뜬다.

한 번 정차할 때 하나의 이벤트만 발생한다. 기어가 `D`가 아니거나 가속 페달을
누르거나 차량이 다시 움직이면 다음 정차 주기를 준비한다. 알림은 engage 여부와
무관하며 LCD에 3초 동안 표시되고, `k230_overlayd`가 보드의 passive piezo에
짧은 PWM 시퀀스를 한 번 재생한다. 선행 차량 출발과 신호 변경은
`signal_changed` 시퀀스를 공유한다.

피에조 알람은 다음 상태 전이에 적용된다.

- `signal_changed`: 선행 차량 출발 또는 신호 변경
- `engage`: 제어 engage 성공
- `disengage`: 제어 disengage 또는 fault에 의한 해제
- `unavailable`: 제어/Panda 상태가 stale이 되거나 Panda/조향 fault가 검출됨
- `unable`: engage 조건을 만족하지 못한 상태에서 engage 명령을 거부할 때

`engage`는 안정적인 상승 다중음, `disengage`는 하강 다중음으로 재생한다.
둘 다 보드 Python 알람과 같은 고정 50% PWM 시퀀스를 공유하므로, PWM duty를
짧게 반복 변경할 때 생기는 클릭/찌그러짐이 없다.

실제 차량/제어 조건으로 engage가 거부되면 LCD 하단에
`UNABLE TO ENGAGE: <사유>`를 3초간 표시하고, openpilot의 refuse 알림에
대응하는 `unable` 피에조 시퀀스를 재생한다. Panda의 `not ready`/`controls off`는
SET edge와 health 응답 사이의 정상적인 비동기 구간이므로 최대 1초 동안 대기한다.
그 사이 Panda 허가가 들어오면 `engage`만 한 번 알린다. 허가가 끝내 오지 않거나
Panda 회복 뒤 다른 정적 조건이 남아 있으면 그때 `UNABLE`로 거부한다.

피에조를 끄거나 핀을 바꾸는 환경 변수(`K230_PIEZO_BUZZER`, `K230_PIEZO_PIN`)와
IOMUX 요건은 [런타임 옵션](runtime-options.md#piezo-alerts)에 있다.

## 판단 근거

신호 대기 판별에 정지선 출력을 쓰지 않는다. 배포 모델(openpilot v0.9.4)이
`stop_lines`를 내지 않기 때문이며, 대신 "모델이 정지 계획"을 근거로 삼는다.
알림 발동 조건은 정지선과 무관하게 모델 경로가 10 m 넘게 열리는 것이라, 무장이
헛나가도 길이 실제로 열리지 않는 한 알림은 뜨지 않는다.

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
