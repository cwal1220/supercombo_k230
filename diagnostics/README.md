# 진단 도구

런타임 경로 밖에 두는 분석·재생·벤치 도구다. 기본 빌드에는 들어가지 않고
`SUPERCOMBO_BUILD_DIAGNOSTICS=ON`일 때 [`CMakeLists.txt`](CMakeLists.txt)가 만든다.
단위 테스트는 [`../gtest/`](../gtest/README.md)에 있다.

## 빌드

호스트 도구는 nncase, OpenCV, K230 디스플레이 라이브러리 없이 빌드된다:

```sh
cmake -S . -B build-host \
  -DCMAKE_BUILD_TYPE=Release \
  -DSUPERCOMBO_BUILD_RUNTIME=OFF \
  -DSUPERCOMBO_BUILD_DIAGNOSTICS=ON
cmake --build build-host --target replay_closed_loop -j2
```

보드 도구는 `build/`의 교차 빌드에 `-DSUPERCOMBO_BUILD_DIAGNOSTICS=ON`을 더해 만든다.
`hud_snapshot`은 `k230_overlayd`처럼 OpenCV가 있는 곳에서만 빌드된다.

## 도구

호스트:

| 도구 | 사용 | 하는 일 |
| --- | --- | --- |
| `replay_closed_loop` | `[옵션] <out.csv\|-> <events.bin...>` | 차 응답을 시뮬레이션해 횡제어 루프를 폐루프로 재생한다 |
| `replay_planner` | `[--laneless] <out.csv> <events.bin...>` | 녹화한 `ModelState`/`ControlState`로 `LateralPlanner`를 다시 돌려 요구 곡률을 CSV로 쓴다 |
| `replay_lateral_learners` | `[옵션] <events.bin...>` | paramsd·torqued 학습기를 녹화에 돌려 학습값을 출력한다 |
| `extract_lateral_dataset` | `<out.csv> <events.bin...>` | `ControlState`마다 CSV 한 행. CAN은 런타임과 같은 `vehicle_can`으로 푼다 |
| `bench_input_warp_overhead` | `[--runs N]` | NV12 → YUV6 입력 경로(직접 패킹, 스칼라·RVV 워프, float·u8)의 CPU 시간. 경로끼리 결과가 다르면 종료 코드 1 |
| `hud_snapshot` | `[--assets DIR] [--model m.bin] [--control c.bin] [--iterations N] [--out PREFIX] [--landscape]` | HUD 시나리오(또는 녹화한 모델·제어 상태)를 `K230ARGB` 프레임으로 그리고 그리기 시간을 출력한다 |

보드:

| 도구 | 사용 | 하는 일 |
| --- | --- | --- |
| `bench_kmodel` | `[--iterations N] <model.kmodel>` | 0으로 채운 입력으로 kmodel 추론 시간을 잰다. 재기 전에 파이프라인을 멈춘다 |
| `run_kmodel_sequence` | `<model.kmodel> <seq.k230msq> <out.bin>` | `K230MSQ1` 입력 시퀀스를 틱마다 넣고 출력을 덤프한다(호스트 FP32와 대조용). 실행마다 CMA가 새므로 강제 종료하지 말고 끝난 뒤 보드를 재부팅한다 |

`check_param_server.py`는 파라미터 서버를 검사하는 Python unittest다. 여기 있지만
`ctest`에 등록돼 `scripts/run_host_tests.sh`로 함께 돈다.

### replay_closed_loop

녹화가 본 차선 기하를 시뮬레이션 차가 벌어진 만큼(`dy`, `dpsi`) 차체 좌표로 다시 돌리므로,
제어를 바꾸면 차가 실제로 다르게 움직인다. 운전자 토크가 있거나 비활성인 틱은 녹화 상태로
재동기화해, 자유 주행 구간마다 실제 자세에서 출발한다.

- 플랜트: `--wn`, `--zeta`, `--delay`, `--gain G`(속도 노드 전부) 또는 `--gain-pts a,b,c,d`
- 컨트롤러: `--sad`(steer_actuator_delay), `--kp`, `--ki`, `--laf`
- 운전자 개입 히스테리시스: `--driver-high`, `--driver-low`, `--driver-release`
- `--open-loop`: 자세 보정을 멈춰 플랜트가 녹화 주행을 얼마나 재현하는지만 본다

재현 점수(전체와 속도 구간별)를 표준 출력으로 낸다. CSV가 필요 없으면 출력 경로에 `-`를
준다. 플랜트 식별과 점수의 해석은 [폐루프 재생](../docs/closed-loop-replay.md)에 있다.

### replay_lateral_learners

controlsd와 같은 `LateralLearners`를 부른다. 활성과 보낸 토크는 `ControlState`에서,
운전자 개입은 기록된 운전자 토크를 컨트롤러와 같게 디바운스해서 얻는다.

- `--steering route/params/steering.json`: 녹화 당시 튜닝(사전값, 지연)을 쓴다. 없으면 코드
  기본값이라 보드와 다를 수 있다.
- `--upstream-schedule`: 조향각·속도를 상류처럼 20 Hz로만 관측한다(기본은 매 틱).
- `--fit-all`: torqued 적합에 점을 전부 쓴다(기본은 상류처럼 무작위 2000점).
- `--torque-cache c.bin`: 있으면 복원하고 저장 틱마다 덮어써, 여러 주행을 런타임 캐시처럼
  잇는다.
- `--inputs`/`--torque-inputs`는 틱마다의 입력을, `--outputs`/`--torque-outputs`는 발행
  메시지를 남긴다. 참조 구현과 대조할 때 쓴다.

## 도구 작성 규칙

- 파일 첫머리의 `/* */` 한글 주석에 용도를 적고 마지막 줄을 `사용:`으로 끝낸다.
- 입력 파일은 위치 인자로, 옵션은 `--이름 값` 또는 값 없는 `--이름`으로 받는다. 모르는
  옵션이 오면 사용법을 출력한다.
- 사용법 오류는 종료 코드 2, 입력을 못 읽거나 결과가 맞지 않는 실행 오류는 1로 끝낸다.

## 자세한 절차

[진단 절차](../docs/diagnostics.md)에 HUD 스냅샷, NV12 재생, 모델 교체 검증, 차선 치우침
분석, 데이터셋 추출, 플래너 재생의 절차와 결과가 있다. 녹화 포맷은
[분할 런타임](../docs/runtime.md#recording-format)에 있다.
