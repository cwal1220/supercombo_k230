# Reproducible quantization evaluation

`evaluate_quantization.py` compares the original 6524-value supercombo output
against a real K230 board dump or the nncase simulator. It normalizes the five
plan logits to plan 0 before comparing them, and reports selected-path error
separately from the error in the same hypothesis. A high whole-vector cosine
similarity alone is insufficient: small logit errors can select another path.

Use `models/verification/quantization_20260903.md` for the measured experiment.
Large local inputs, intermediate ONNX models, kmodels and raw dumps live in
`models/quantization_work/` (ignored by Git).
The selected model is also preserved as `models/supercombo.kmodel`; its exact
173-sample NPZ is committed as `models/ptq/supercombo_calib_mixed173.npz`.
`scripts/build_supercombo_model.sh` rebuilds this default with centering and
SQuant. The original 77-sample NPZ remains available for the comparisons below.

## Native K230 recordings

Build the host warp helper from the same production preprocessing sources:

```sh
cmake -S . -B build-quant-host -DSUPERCOMBO_BUILD_RUNTIME=OFF \
  -DSUPERCOMBO_BUILD_BENCHMARKS=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build-quant-host --target warp_recording -j2
```

Create a JSON manifest containing objects such as
`{"video": "/absolute/route/segments/003/road.hevc", "start_frame": 120}`.
Choose separate **routes** for calibration, tuning and final evaluation.
The loader checks the recording index, reads recorded desire edges, decodes
NV12 directly with FFmpeg, applies the production fixed12 dual-view warp and
warms the original FP32 recurrent state before collecting samples.
Frame counts are not duration estimates: use capture timestamps, since these
recordings advertise 20 fps but their measured median interval is about 64 ms.

```sh
python tools/model/make_k230_quant_dataset.py \
  --manifest models/quantization_work/train_manifest.json \
  --model /path/to/original/supercombo.onnx \
  --warp-binary build-quant-host/bin/warp_recording \
  --out models/quantization_work/train_k230.npz \
  --warmup 40 --frames 40 --stride 10
```

For continuous evaluation, use `--stride 1` (the default). For calibration,
include actual left/right lane-change transitions and preserve the pulse frame;
sparse sampling can miss these one-frame inputs. This tool currently supports
the 1280x720 camera geometry and ControlState prefixes of recording v2–v5.
It rejects missing/empty calibration JSON instead of silently guessing.

To reconstruct the selected mixed calibration set, use the committed
`models/verification/quantization_20260903/train_manifest.json` with
`--warmup 40 --frames 40 --stride 10` (72 samples), then `pulses_manifest.json`
in the same directory with `--warmup 40 --frames 12 --stride 1` (24 samples).
The manifests contain the recording paths used for this experiment; adjust
their root if the recordings are stored elsewhere. Save the two outputs as
`models/quantization_work/train_k230.npz` and `pulses_k230.npz`, then merge:

```python
import numpy as np
from pathlib import Path

root = Path('models')
parts = [np.load(root / p, allow_pickle=False) for p in (
    'ptq/supercombo_calib.npz',
    'quantization_work/train_k230.npz',
    'quantization_work/pulses_k230.npz')]
merged = {}
for name in ('input_imgs', 'big_input_imgs', 'desire',
             'traffic_convention', 'initial_state'):
    values = np.concatenate([part[name] for part in parts])
    if name.endswith('imgs'):
        assert np.array_equal(values, np.round(values))
        assert values.min() >= 0 and values.max() <= 255
    merged[name] = values.astype(np.uint8 if name.endswith('imgs') else np.float32)
assert len(merged['desire']) == 173
np.savez_compressed(root / 'quantization_work/rebuilt_mixed173.npz', **merged)
for part in parts:
    part.close()
```

The mixture intentionally has no `sequence_id`: unrelated PTQ samples must
not be treated as a continuous drive. The committed metadata records exact
source NPZ hashes and each native sample. Re-decoding or changing numerical
dependencies may change regenerated tensors; use the preserved NPZ for the
exact measured compiler input.

The route's saved pitch/yaw snapshot is used with roll zero. HEVC compression and
online calibration changes prevent exact reproduction of the online ISP input.
The same decoded tensors feed FP32 and K230, isolating model conversion error
within this offline comparison. These metrics are not a road safety validation.

## Independent and recurrent comparison

Independent mode supplies each saved teacher state to both models. Recurrent
mode starts each continuous clip at the saved warm state, then lets each model
feed its own last 512 outputs back. Do not concatenate unrelated PTQ samples
into a recurrent sequence. The NPZ must contain `sequence_id` for this mode.

```sh
python tools/model/evaluate_quantization.py run \
  --model /path/to/original/supercombo.onnx \
  --data models/quantization_work/validation_k230.npz \
  --out models/quantization_work/reference.npy --recurrent

python tools/model/make_supercombo_sequence.py \
  --npz models/quantization_work/validation_k230.npz \
  --out models/quantization_work/validation.bin \
  --mode uint8 --frames 160 --state-mode recurrent
```

Build `sequence_runner` with the K230 toolchain and
`SUPERCOMBO_BUILD_BENCHMARKS=ON`. On an available, stationary board, execute the
runner in a separate experiment directory after stopping managed inference
and control processes. It has no CAN or IPC output:

```sh
./sequence_runner candidate.kmodel validation.bin candidate.bin
```

The new SCSEQ1 mode 3 preserves uint8 image tensors and adds, after desire and
traffic, a little-endian `uint32 reset` and 512 float32 state values per frame.
Independent mode sets every reset to 1; recurrent mode sets it only at clip
boundaries. Existing modes 0–2 remain supported.

Copy the raw dump back and compare:

```sh
python tools/model/evaluate_quantization.py compare \
  --reference models/quantization_work/reference.npy \
  --candidate models/quantization_work/candidate.bin \
  --out models/quantization_work/metrics.json
```

The nncase simulator is also available via `bash tools/model/run_nncase.sh
python tools/model/evaluate_quantization.py run ...`. The evaluator locates
the KPU simulator executable and uses a temporary working directory for its
`gmodel_dump_dir`. On this Mac the simulator is much slower than the board.

## Model and compiler options

The original probability biases share an approximately -320 offset. The KPU
uses FP16 intermediate outputs even with INT16 activation quantization; at
that magnitude its spacing is 0.25. `--plan-prob-center-before-delta` in
`rewrite_supercombo_onnx.py` subtracts the common bias **before** Gemm's output
rounding, then still emits the existing plan-0 deltas. This preserves the
runtime output layout and all non-logit outputs in FP32.

`--plan-prob-delta` instead subtracts weights too. It is a separate experiment:
it changes the weight quantization error and must not be assumed superior.

`compile_supercombo_nncase.py` supports `NoClip`/`Kld`,
`--finetune-weights-method UseSquant`, and explicit mixed
quantization schemes. nncase 2.11 exposes `use_mse_quant_w` in its option object
but does not forward it to the compiler: the experimental kmodel was byte-for-byte
identical. The script now rejects this silently ignored flag on affected bindings.
Available knobs are not evidence of improvement; compare
the resulting kmodel. Both activation and weight types cannot be INT16 on K230.
`--export-weight-range-by-channel` exports ranges; it is not by itself a switch
that proves per-channel quantization was enabled.

The helper now rejects empty/mismatched sample counts, nonfinite inputs,
invalid shapes and lossy casts to integer model inputs. Run its checks with:

```sh
python tools/model/test_quantization_tools.py
```

The Docker wrapper carries the Rosetta/.NET flags used by this workspace and
disables core dumps. An occasional empty-log startup SIGSEGV can require a
retry. nncase 2.11 IR diagnostics export `QuantScheme.json`, but compilation
with IR dumping can subsequently fail in the Flatten metric evaluator; use
`--no-dump-ir --no-dump-asm` for the actual artifact.

Official references: [nncase mixed quantization](https://github.com/kendryte/nncase/blob/master/docs/MixQuant.md),
[K230 PTQ options and precision constraints](https://www.kendryte.com/k230_rtos/en/v0.8/app_develop_guide/ai/nncase.html),
[simulator PATH troubleshooting](https://github.com/kendryte/nncase/blob/master/docs/FAQ_EN.md).
