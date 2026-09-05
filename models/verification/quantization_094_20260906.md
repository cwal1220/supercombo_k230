# v0.9.4 supercombo 양자화 열화 저감 (2026-09-06)

main(c2 모델)에서 확정한 레시피를 v0.9.4 supercombo에 적용하고 실제 K230에서 검증했다.
기준은 배포 kmodel(60장 PTQ, SHA `61c36027…`, md5 `697745a1`)이고, 같은 스크립트로 재컴파일하면
md5가 같아 파이프라인 재현성은 먼저 확인했다. 결과 kmodel은 SHA `065bf2a8…`(md5 `d0a9e692`), 27.43 MiB, 27.7 ms/frame.

보드 600프레임 open-loop에서 plan 가설 일치 92.5→94.5 %, 선택 경로 y MAE 6.1→5.1 cm, plan x MAE 0.98→0.91 m.
closed-loop 100프레임(런타임 end-to-end)에서 차선 y MAE 0.129→0.102 m, 경로 y MAE 0.086→0.075 m. 추론 시간은 동일하다.

## main과 다른 점

- 그래프: Conv 120, Elu 79, Gemm 79, Relu 65. Tanh/Sigmoid가 없어 `Tanh → 2·Sigmoid(2x)−1` 재작성은 해당 없음.
  GRU 대신 출력 5990:6118을 `features_buffer`로 되먹인다.
- plan 가설 확률 로짓의 bias가 −9.5 근방이라 로짓 중심화도 불필요.
- 따라서 적용한 레버는 둘이다: 가중치 사전 양자화 + 순차 bias correction(`tools/model/prequant_bias_correct.py`,
  by_channel 스킴 `models/ptq/supercombo_quant_scheme_bychannel.json`, 96장), 보정 데이터 60→180장
  (`ptq/supercombo_calib.npz` 60 + `ptq/supercombo_calib_k230_120.npz` 120, 7개 K230 루트).

## 평가 데이터

- open-loop 600: 6개 루트의 held-out 구간(보정에 쓰지 않은 세그먼트) 각 100프레임 연속.
  `day_0816`(고속 60 km/h), `day_0830hw`(고속도로), `eve_0830`, `eve_0903`(저녁 시내), `night_0830a`(야간), `night_0901`(야간 정차).
  FP32 러너로 워밍업한 뒤 프레임별 FP32 desire/features_buffer를 주입하므로 변환 오차만 측정한다(`run_kmodel_sequence`, K230MSQ1).
- closed-loop 100: `eve_0903` 루트 SCNV12 replay를 보드 `k230_modeld`(`SUPERCOMBO_REPLAY_NV12`, `SUPERCOMBO_CALIB_AUTO=0`)로 재생,
  워프·시간 입력·모델을 모두 보드가 처리한 출력을 호스트 FP32와 비교한다.
- 시뮬레이터(`nncase.simulator.k230.sc`)와 보드 출력은 비트 동일(배포 모델로 확인). 수치는 모두 보드 실측.

## 후보 비교 (보드 open-loop 600프레임, FP32 대비)

| 후보 | plan 일치 | plan y 2 s | plan y 평균 | plan x | lane y | lead x | 전체 MAE |
|---|---:|---:|---:|---:|---:|---:|---:|
| 배포(60장) | 0.925 | 0.0197 | 0.0615 | 0.980 | 0.155 | 0.994 | 0.0644 |
| 보정 270장 | 0.925 | 0.0199 | 0.0624 | 1.005 | 0.158 | 0.966 | 0.0657 |
| SQuant + 270장 | 0.930 | 0.0206 | 0.0558 | 0.890 | 0.156 | 0.803 | 0.0602 |
| 사전양자화 BC(보정 세트로) + 270장 | 0.948 | 0.0225 | 0.0625 | 0.912 | 0.181 | 1.025 | 0.0664 |
| 사전양자화 BC(넓은 풀 96장) + 270장 | 0.950 | 0.0203 | 0.0494 | 0.893 | 0.155 | 0.940 | 0.0650 |
| **빌드 스크립트(BC 96 + 180장)** | 0.945 | 0.0198 | 0.0509 | 0.914 | 0.159 | 0.977 | 0.0654 |

루트별 plan y 2 s MAE (일치율) / lane y MAE, 배포 → 빌드:

| 루트 | 배포 | 빌드 |
|---|---|---|
| day_0816 | 0.0047 (100 %) / 0.093 | 0.0076 (100 %) / 0.120 |
| day_0830hw | 0.0151 (75 %) / 0.100 | 0.0105 (85 %) / 0.067 |
| eve_0830 | 0.0103 (96 %) / 0.084 | 0.0110 (95 %) / 0.066 |
| eve_0903 | 0.0214 (96 %) / 0.114 | 0.0251 (99 %) / 0.082 |
| night_0830a | 0.0188 (88 %) / 0.094 | 0.0172 (88 %) / 0.089 |
| night_0901 | 0.0483 (100 %) / 0.445 | 0.0471 (100 %) / 0.528 |

전체 MAE는 로짓·std 슬라이스가 지배해 후보 간 차이가 거의 없고, 제어에 쓰이는 plan/lane 슬라이스가 갈린다.
lane y 평균의 절반은 정차 야간 루트(`night_0901`, 차선 없음)에서 나온다.

## closed-loop (보드 런타임 end-to-end, eve_0903 100프레임)

| 지표 | 배포 | BC 96 + 270장 | 빌드 |
|---|---:|---:|---:|
| plan 일치 | 0.82 | 0.84 | 0.84 |
| plan y 2 s MAE / 최대 | 0.030 / 0.190 | 0.029 / 0.189 | 0.029 / 0.183 |
| plan y 평균 MAE | 0.086 | 0.073 | 0.075 |
| lane y MAE | 0.129 | 0.105 | 0.102 |
| edge y MAE | 0.220 | 0.207 | 0.200 |
| feature 프레임 간 상관(FP32 0.621) | 0.600 | 0.599 | 0.601 |
| 전체 MAE | 0.0849 | 0.0777 | 0.0788 |

## 원인 분리

가중치 uint8 채널별 반올림만 호스트에서 에뮬레이션하면 전체 MAE ≈ 0.023, 보드 실측은 0.065다.
잔여의 약 2/3는 KPU 커널 수치, 그중 ELU 테이블(x∈[−2,0)에서 +0.012 평균 편향, main 프로브와 동일)이 주다.
ELU 79개를 Relu/Exp로 분해하면 정확하지만 NPU 시간이 4.6배라 기각했다.

## 기각된 레버

- 보정 데이터만 270장으로 확대: 변화 없음(nncase 기본이 이미 채널별, NoClip 범위는 60장에서 포화).
- SQuant: 전체 MAE 최저지만 plan y 2 s는 나빠지고 plan 일치는 거의 안 오른다.
- BC 입력을 보정 세트 자체(270장)에서 뽑은 경우: plan 일치는 오르지만 lane y가 0.181로 악화. BC 샘플은 루트를 넓게 섞은 96장이 낫다.
- main에서 확인된 무효 옵션(`UseAdaRound`, `use_mse_quant_w`, 부분 int16 가중치)은 재시험하지 않았다.

## 재현

`PYTHON_BIN=<onnxruntime venv> scripts/build_supercombo_model.sh` → `models/supercombo.kmodel` SHA `065bf2a8…`.
실험 디렉터리 `models/work/qexp094/`(git 무시): `make_eval094.py`(평가 세트), `write_msq.py`, `board_eval094_all.sh`(보드 open-loop),
`metrics094.py`/`metrics_all094.py`(지표), `exp_compile.py`/`run_exp.sh`(후보 컴파일), `replay_eve_0903/`(closed-loop replay).
