# Supercombo 양자화 열화 실측 및 개선 — 2026-09-03

`6c86a4a` 시점의 브랜치 모델을 기준으로 양자화 열화를 실제 K230에서 재측정했다. 검증 구간에서
**공통 plan 편향 제거 + C2/K230 혼합 173개 PTQ 샘플 + SQuant**를 선택했고,
보정·선택에 쓰지 않은 9월 3일 주행 640프레임에서도 개선을 확인했다.
후보는 별도 파일로 보존했다. 보드 운영 모델과 관리 프로세스는 원래 상태로 복구했다.

**후속 반영:** 선택된 SQuant 후보를 `models/supercombo.kmodel` 기본 모델로 승격했다.
정식 보정 데이터는 `models/ptq/supercombo_calib_mixed173.npz`에 보존한다.
아래 표의 “현재 브랜치”는 측정 당시 `6c86a4a` 모델을 뜻하며, 변경 후 기본 모델은 “선택 후보”다.

## 모델 식별

- 작업 저장소: `supercombo_k230`, 브랜치 `main`, 시작 커밋 `6c86a4a`.
- 원본: `openpilot_c2/selfdrive/modeld/models/supercombo.onnx`, 약 2,351만 파라미터.
- 원본 SHA-256: `50c7fc8565ac69a4b9a0de122e961326820e78bf13659255a89d0ed04be030d5`.
- 브랜치 kmodel SHA-256: `168e6a15886301250b9b554a04156ad0a399a04f5bd93b97ea0a5c72c0eba05d`.
- 보드에서 원래 실행 중이던 별도 모델 SHA-256:
  `61c360274a10980378c9518e4eac53591798a1b64c1a161a896280a3ac658a27`.
  이 모델을 브랜치 모델로 오인하지 않았으며, 운영 파일을 교체하지 않았다.
- 컴파일러: nncase 2.11.0 / nncase-kpu 2.11.0, K230 target.
- 현재 브랜치 PTQ: INT16 activation, UINT8 weight, NoClip, C2 보정 77개.
- 입력: `input_imgs`, `big_input_imgs` 각 `[1,12,128,256]` uint8,
  `desire [1,8]`, `traffic_convention [1,2]`, `initial_state [1,512]` float32.
- 두 영상 타워와 원래 GRU 업데이트 식을 유지한다. 기존 문서의 “no-GRU”는
  GRU 자체 제거가 아니라 `--gru-update` 재작성 옵션을 쓰지 않았다는 뜻이었다.
- 출력: FP32 6524개. 마지막 512개를 다음 프레임 state로 되먹임한다.

기존 `openpilot_c2_master_comparison.json`은 다른 kmodel·PTQ 해시의 과거 보고서다.
이번 브랜치 모델의 기존 C2 보정 77개 재측정은 후보 선택 **61/77**, 최고점 동점 **23개**,
비-logit 출력 MAE **0.054438**, 선택 경로 y MAE **0.185830m**였다.
이 77개는 보정 데이터이므로 독립 검증 성능으로 해석하지 않는다.

## 확인된 원인과 변경

1. 원래 plan 점수의 편향에는 약 -320의 공통값이 있다. INT16 PTQ를 사용해도
   K230 컴파일 결과에는 FP16 중간 출력이 있으며, 이 크기에서 간격은 0.25다.
   기존 그래프는 큰 점수를 반올림한 뒤 plan 0을 빼므로 작은 후보 간 차이를 잃었다.
   보드에서 실제로 0.25 단위의 점수와 다수의 동점을 확인했다.
2. `--plan-prob-center-before-delta`는 다섯 bias의 공통 평균을 Gemm 전에 빼고,
   기존과 동일하게 plan-0 대비 delta를 출력한다. 가중치와 나머지 출력 배치는 유지한다.
   독립 FP32 240프레임에서 비-logit 출력 차이 **0**, 후보 선택 **240/240** 일치,
   점수 delta 최대 차이 **0.0000301**이었다. FP32 반올림 수준의 차이만 남았다.
3. 원래 77개 C2 샘플에 실제 K230 카메라의 96개를 추가했다. 낮·야간, 정차·주행,
   실제 좌/우 차선변경 pulse를 포함했다. native K230 샘플만 쓰면 범위가 부족해
   오히려 열화가 커졌다. 넓은 C2 입력 범위와 K230 입력 분포를 함께 유지했다.
4. SQuant를 켠 실제 kmodel을 비교했다. 가중치 보정은 입력에 따른 오차를 바꾸므로
   항상 개선된다고 가정하지 않고 동일 입력과 분리된 주행으로 평가했다.

[공식 nncase PTQ/정밀도 옵션](https://www.kendryte.com/k230_rtos/en/v0.8/app_develop_guide/ai/nncase.html)은
NoClip/KLD 및 SQuant를 제공한다. K230은 활성값과 가중치를 모두 INT16으로 설정할 수 없다.

## 데이터와 평가 조건

- 보정: 기존 C2 77개 + K230 일반 구간 72개 + 실제 차선변경 주변 24개 = **173개**.
- K230 보정 주행: `2026-08-16--11-51-52-654`, `2026-08-30--00-51-57-534`,
  `2026-08-30--20-58-46-660`. 일반 구간은 18개 clip에서 4개씩,
  차선변경은 실제 desire 3/4 전이가 있는 2개 clip에서 12개씩 수집했다.
- 선택용 검증: 별도 route `2026-08-30--01-24-02-971`, 4개 연속 clip / **160프레임**.
- 최종 검증: 별도 route `2026-09-03--21-08-52-823`, 6×40 + 2×200 = **640프레임**.
  두 긴 clip은 짧은 clip과 프레임이 겹치지 않는다. KLD를 포함한 후보 순위는
  선택용 검증 지표로 판단했다. 최종 주행은 선택된 SQuant 후보의 확인에 사용했다.
- 보정 JSON이 비어 있던 9/3 새벽 route와 1280×704 geometry의 옛 영상은 제외했다.
- FFmpeg가 HEVC를 NV12로 직접 디코딩한다. RGB 왕복 변환 없이 실제
  `ModelInputTransform` C++ fixed12 코드로 med/sbig 두 시점을 각각 변환한다.
- 인접한 녹화 프레임을 previous/current로 묶고, 원본 FP32를 40프레임 실행해
  state를 준비한다. desire는 기록된 ControlState에서 rising edge로 복원한다.
- 독립 비교: 매 샘플의 같은 저장 state를 두 모델에 입력한다.
  연속 비교: clip 시작에만 같은 저장 state를 넣고 각 모델의 자체 state를 되먹임한다.
- 녹화 헤더는 20fps지만 실제 중간 간격은 약 64ms다. 200프레임은 약 12.8초이며,
  드문 100ms 전후 간격도 있다. 시간을 20fps로 가정해 해석하지 않았다.
- HEVC 손실압축과 route의 고정 pitch/yaw snapshot을 쓰므로 온라인 ISP 및
  시간에 따라 갱신되는 calibration을 완전히 재현하지 않는다. 동일한 오프라인
  입력을 두 모델에 넣어 변환으로 인한 차이를 측정한 것이다.
- 실측 도구 `sequence_runner`는 CAN/IPC에 데이터를 발행하지 않는다.

정확한 route/시작 프레임은 [실험 manifests](quantization_20260903/train_manifest.json),
[입력 분포 및 기록 간격 검사](quantization_20260903/dataset_audit.json)에 보존했다.

## 방법별 실측 — 선택용 검증, 연속 추론 160프레임

| 방법 | 보정 수 | 후보 일치 | 전체 출력 RMSE¹ | 선택 경로 y MAE (cm) | state MAE |
|---|---:|---:|---:|---:|---:|
| 현재 브랜치 | 77 | 153/160 | 0.41470 | 10.845 | 0.010523 |
| 가중치·편향 차분을 Gemm 전에 계산 | 77 | 151/160 | 0.41470 | 11.086 | 0.010523 |
| 공통 편향만 먼저 제거 | 77 | 157/160 | 0.41470 | 10.691 | 0.010523 |
| 편향 제거 + 혼합 보정 | 173 | 157/160 | 0.40989 | 10.739 | 0.010472 |
| 편향 제거 + K230 데이터만 보정 | 96 | 122/160 | 1.06155 | 16.219 | 0.012456 |
| 편향 제거 + 혼합 보정 + SQuant | 173 | 156/160 | 0.34241 | 9.187 | 0.010305 |
| 편향 제거 + 혼합 보정 + KLD | 173 | 134/160 | 0.66708 | 14.458 | 0.045567 |

공통 편향 제거만으로도 동점과 후보 선택이 개선된다. SQuant 혼합 후보는
전체 출력·선택 경로·가까운 경로 오차를 함께 줄여 선택했다. 가중치 차분 방식,
K230 데이터만 사용한 방식, KLD는 더 나쁜 지표가 있어 채택하지 않았다.

`use_mse_quant_w=True`도 시험했으나 nncase 2.11 Python binding이 이 값을
컴파일러에 전달하지 않았다. 결과는 혼합 NoClip 모델과 **SHA-256까지 동일**했다.
이를 별도 개선 실험으로 세지 않았고, 해당 binding에서 이 옵션을 요청하면
오류를 내도록 바꿔 조용한 무효 설정을 막았다.
KLD는 병렬 실행 중 Docker OOM으로 종료됐으나 단독 재컴파일 후 보드 측정까지 완료했다.

## 최종 별도 주행 — 연속 추론 640프레임

| 지표 | 현재 브랜치 | 선택 후보 | 변화 |
|---|---:|---:|---:|
| 경로 후보 일치 | 555/640 (86.72%) | 590/640 (92.19%) | +5.47%p |
| 최고 점수 동점 | 111 | 3 | 감소 |
| 전체 출력 MAE¹ | 0.067577 | 0.061901 | -8.40% |
| 전체 출력 RMSE¹ | 0.314407 | 0.270548 | -13.95% |
| 경로 후보 점수 차이 MAE | 0.174476 | 0.137574 | -21.15% |
| 선택 경로 y MAE (cm) | 8.029368 | 6.940979 | -13.56% |
| 선택 경로 y P95 (cm) | 42.675937 | 36.266348 | -15.02% |
| 선택 경로 y 최대 (m) | 1.646351 | 1.353574 | -17.78% |
| 앞 17개 knot y MAE (cm) | 0.738444 | 0.704844 | -4.55% |
| 차선 좌표 MAE (m)² | 0.064065 | 0.052930 | -17.38% |
| 리커런트 state MAE | 0.011445 | 0.011262 | -1.60% |
| pose MAE | 0.026555 | 0.026883 | +1.23% |
| pose 최대 오차 | 1.060980 | 1.423243 | +34.14% |
| desire logit MAE | 0.237670 | 0.240435 | +1.16% |
| 순수 추론 평균 (ms) | 27.860 | 27.810 | 동등 수준 |

¹ 원래 점수는 절대 logit, K230은 상대 logit이므로 다섯 점수는 별도로 비교한다.
“전체 출력” 수치는 나머지 6519개 값이며 서로 다른 물리량이 섞인 수치 오차다.
실제 차선 유지 성공률이나 차량 횡방향 추종 오차를 의미하지 않는다.
² 차선 head의 평균 좌표 부분만 비교한다. 전체 head에는 log-standard-deviation도 있다.

개선 후보에서도 **50프레임의 후보 선택 불일치**가 남았다. state MAE 개선은 약 1.6%에
그쳤고 두 긴 구간의 후반 state 오차는 초기보다 높았다. pose MAE는 약 1.23%,
desire logit MAE는 약 1.16% 나빠졌으며 pose 최대 오차는 1.061→1.423으로 증가했다.
선택 경로 x 최대 오차도 62.520→62.832m로 소폭 증가했다. 모든 출력과 최악값이
개선된 모델은 아니며, 양자화 열화가 제거됐다고 판단하지 않는다.

## 결과 파일과 재생성

- [선택 후보 kmodel](../quantization_work/recommended/supercombo.kmodel)
- [컴파일한 ONNX](../quantization_work/recommended/supercombo.onnx)
- [173개 보정 NPZ](../quantization_work/recommended/calibration.npz)
- [후보 SHA-256 manifest](../quantization_work/recommended/manifest.sha256)
- [모든 지표·모델/데이터 해시 JSON](quantization_20260903.json)
- [평가·데이터 생성 도구 설명](../../tools/model/QUANTIZATION.md)

후보 kmodel SHA-256:
`3bc1c91da706f99245168a6e231d17609a5b1b8b23d7ad1aabb70b749293d8a9`.

저장된 보정 세트로 별도 디렉터리에 재생성:

```sh
PYTHON_BIN=../.model-venv/bin/python \
SOURCE_ONNX=/Users/chan/Documents/openpilot_c2/selfdrive/modeld/models/supercombo.onnx \
CALIB_NPZ=models/quantization_work/recommended/calibration.npz \
CALIB_SAMPLES=173 \
FINETUNE_WEIGHTS_METHOD=UseSquant \
CANDIDATE_DIR=models/quantization_work/rebuilt \
  scripts/build_quantization_candidate.sh
```

후보 생성 스크립트는 원래 GRU 식, INT16/UINT8, NoClip을 사용하고 위 centering을 적용한다.
보정 재생성 명령과 sequence format은 도구 설명에 있다. 원래 브랜치 모델·ONNX·PTQ
해시는 작업 전후 동일하다. 후보는 별도 실험 디렉터리에서만 실행했다.

검증: 실제 K230의 모든 후보 독립/연속 추론, FP32 구조 동등성, host warp 빌드,
입력 형식·오차 계산의 5개 테스트, Python/Shell 문법 및 diff 공백 검사 완료.
보드 manager, camerad/modeld/controlsd/pandad/recordd/overlayd, parameter server 재실행과
원래 운영 모델 해시를 확인했다. 복구 후 modeld는 약 19.8fps, 오류 0으로 실행 중이었다.
