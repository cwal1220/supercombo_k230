# 호스트 단위 테스트

보드 없이 호스트에서 도는 googletest 테스트다. 파일 하나가 실행 파일 하나이고
(`gtest_<이름>.cc` → `build-host/bin/gtest_<이름>`), 교차 빌드에는 들어가지 않는다.

## 실행

```sh
./scripts/run_host_tests.sh
```

`build-host`를 구성하고 `host_tests` 타깃을 빌드한 뒤 `ctest`로 전부 돌린다(C++ 83개와
Python 1개). googletest v1.18.0은 첫 구성 때 받아 온다(SHA256 고정).

하나만 돌릴 때:

```sh
ctest --test-dir build-host -R ControlReplay.InactiveDesiredTracksActual --output-on-failure
build-host/bin/gtest_lateral_learners --gtest_filter='LateralLearners.Torque*'
```

테스트는 `params/`를 상대 경로로 읽으므로 저장소 루트에서 돌아야 한다. `ctest`는 루트에서
돌리고, 실행 파일을 직접 부를 때도 루트에서 부른다.

## 목록

| 실행 파일 | 개수 | 검사 내용 |
| --- | ---: | --- |
| `gtest_adaptive_cruise` | 14 | 비전 크루즈 버튼 간격과 한계. 차량 모형과 폐루프로 돌려 설정 속도 동기화, 재설정, 반응 없는 차, 오르내림 반복을 본다 |
| `gtest_calibration_equivalence` | 4 | 온라인 보정 상태 기계(calibrationd.py 참조), 저장·복원·수동 보정, 환경 변수, 투영 행렬과 YUV6 워프(openpilot OpenCL 참조) |
| `gtest_can_queue` | 1 | 공유 메모리 CAN 큐 |
| `gtest_control_replay` | 36 | K7 CAN 신호 해석, engage 게이트와 홀드, 토크 한계와 MDPS 고장 회피, CAN 프레임 구성, 학습값 소비(끄면 비트 동일, opendbc `calc_curvature`, latAccelFactor, `paramsd_invalid`, 롤 반영 곡률 한계), CAN 픽스처 재생 |
| `gtest_departure_alert` | 5 | 정차 중 앞차 출발과 신호 대기 알림 |
| `gtest_lateral_learners` | 13 | paramsd EKF(야코비안, Joseph 양정치, 수렴, 게이트, 출력 한계, 저장, 자이로 바이어스), torqued(TLS와 닫힌 해 대조, 버킷, 게이트, 필터·decay, 4 Hz/12 s 스케줄, 캐시), controlsd 연결 |
| `gtest_lateral_mpc` | 1 | 횡 MPC 최적성. 동역학과 코스트를 따로 구현해 시나리오 8개의 수렴점에서 기울기를 본다([검증 기록](../docs/verification.md#lateral-mpc-solver)) |
| `gtest_model_output_parser` | 2 | supercombo raw 출력 레이아웃과 시간축 입력 규약 |
| `gtest_overlay_state` | 5 | 제어 상태 → HUD 매핑, 모든 `BlockReason`의 라벨, 알림 선택(기준값, 카운터 리셋, 우선순위) |
| `gtest_panda_can_codec` | 1 | panda USB CAN 패킹·언패킹 |
| `gtest_recording_writer` | 1 | 디스크의 route 구조: K230LOG1 청크, K230IDX1 인덱스, 매니페스트, params 스냅샷, 스테이징 비우기 |

`diagnostics/check_param_server.py`(Python unittest)도 `ctest`에 등록돼 함께 돈다.
파라미터 저장소, 런타임 스키마 동기화, `params/*.json`의 UI 메타데이터, UI min/max와 C++
`Json*Field` 클램프 표의 일치를 본다.

## 인자를 주면 도는 모드

- `gtest_control_replay <fixture.k230can>`: `ControlReplay.CanFixture`가 녹화한 CAN을
  컨트롤러에 흘려 목표 곡률을 openpilot 참조식과 대조하고, LKAS/MDPS/CLU11 주기와 토크
  범위를 본다. 인자가 없으면 건너뛴다. 픽스처는 `tools/control/export_can_fixture.py`가
  녹화 `events/NNN.bin` 하나로 만든다. 60초 연속 주행 구간이어야 하며(활성 5900틱 초과,
  토크 > 0) 정차 구간은 실패한다.
- `gtest_model_output_parser <SCODMP1 덤프>`: 테스트 대신 첫 프레임을 파싱해 출력한다.
  덤프는 `k230_modeld`를 `SUPERCOMBO_RAW_DUMP`로 돌려 만든다
  ([런타임 옵션](../docs/runtime-options.md)).

## 테스트 추가

1. `gtest/gtest_<이름>.cc` 하나에 테스트와 도우미를 모두 둔다. 공유 헤더는 두지 않는다.
2. [`CMakeLists.txt`](CMakeLists.txt)에 한 줄을 더한다:
   `add_host_test(gtest_<이름> <라이브러리...>)`. 라이브러리는 루트 CMake의 `common`,
   `control_core`, `planning`, `perception`, `panda_codec`, `recording_writer`,
   `overlay_state` 중에서 고른다.
3. 작성 규칙:
   - 파일 첫머리에 `/* */` 한글 주석으로 무엇을, 무엇과 대조해 검사하는지 적는다.
   - TEST와 도우미는 익명 namespace 안에 둔다.
   - 주석과 실패 메시지는 한글로 쓴다. 단언 묶음의 뜻은 위에 `//` 한 줄로, 단언 하나의
     뜻은 `<< "..."`로 붙인다. TEST 위 주석은 이름만으로 모자랄 때만 단다.
   - 실패할 때 값이 보이는 단언을 쓴다. `ASSERT_TRUE(a < b)` 대신 `ASSERT_LT(a, b)`,
     `ASSERT_TRUE(std::fabs(a - b) < t)` 대신 `ASSERT_NEAR(a, b, t)`.
   - 값을 돌려주는 도우미에서는 `ASSERT_*`를 못 쓰니 `ADD_FAILURE() << ...; return {};`로
     실패를 남긴다.
   - 통과할 때는 아무것도 출력하지 않는다. 판정 없는 시간 측정은
     [`diagnostics/`](../diagnostics/README.md)의 벤치로 만든다.
