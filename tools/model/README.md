# Model tools

Scripts that turn an upstream openpilot `supercombo.onnx` into the K230
`.kmodel`. `../../scripts/build_supercombo_model.sh` runs them in order; see
`../../models/README.md` for the resulting package, its contract, and the board
verification numbers.

## Scripts

- `sanitize_onnx_for_nncase.py`
  - Normalizes the release ONNX for nncase: fp16 weights and tensors to fp32
    (the PTQ path needs an fp32 graph), duplicate opset entries merged, empty
    node names filled, and `Reshape` `allowzero` removed. Values are unchanged.

- `retype_image_inputs_uint8.py`
  - Retypes the image inputs to uint8 and inserts `DequantizeLinear(scale=1)`.
    Bit-identical outputs, but the runtime writes a quarter of the bytes and
    skips the int-to-float conversion.

- `compile_supercombo_nncase.py`
  - Imports the ONNX into nncase, applies PTQ calibration, and writes the
    `.kmodel`.
  - On this Mac the practical K230 path is the `linux/amd64` Docker image
    `supercombo-nncase-k230:2.11.0-sdk`. The build script passes the
    Rosetta/.NET mitigation flags; without them the compiler spins on the ONNX
    import and then dies inside the .NET JIT.

## Host environment

```sh
python3 -m venv ~/Documents/k230/.model-venv
~/Documents/k230/.model-venv/bin/pip install -r tools/model/requirements.txt
```

## Recording-driven helpers

These read `k230_recordd` routes and reproduce the device's input pipeline
(`k230_route.py` decodes the route, `model_warp.py` is a numpy port of
`src/model_input_transform.cc`, `route_frames.py` joins them, and
`op094_runner.py` drives the model with the same desire/feature history the
runtime keeps).

- `make_calibration.py`
  - captures PTQ samples across routes; each sample carries the feature buffer
    the model itself produced, so calibration sees the real activation ranges.

- `make_replay.py`
  - writes an `SCNV12R1` replay plus host reference outputs for the board
    verification described in `../../docs/diagnostics.md`.

- `lane_bias.py`
  - measures the lateral bias of a drive and splits it into a translation term
    and a rotation term, which is what tells you whether a lane-hugging
    complaint is a camera-calibration problem or not. See
    `../../docs/diagnostics.md`.
