# Verification

[← Documentation index](../README.md)

## Calibration and input-warp equivalence

- `benchmarks/verify_calibration_equivalence.cc` is a host-only verifier for the
  openpilot-derived calibration and input-warp math. It checks the pose-based
  calibration state machine, manual-vs-online feedback policy,
  medmodel/sbigmodel homography matrices, UV `transform_scale_buffer(0.5)`
  handling, and YUV6 plane order (`Y00, Y10, Y01, Y11, U, V`) without requiring
  nncase or K230 display libraries.
- The verifier intentionally treats model-input feedback as roll-free, matching
  openpilot's `get_view_frame_from_road_frame(0, pitch, yaw, model_height)`
  extrinsic matrix. `rpyCalib` may contain a tiny roll internally in openpilot,
  but that roll is not fed into `modeld`.
- The verifier checks the ISP-normalized medmodel defaults and compares its
  matrix with the original openpilot formula.
- The verifier compares every output byte from the compact lookup tables with the
  previous 24-byte/sample lookup format across 12 medmodel/sbigmodel pose cases.
  It also checks two-frame direct tensor history against the previous pack path
  with a full-buffer `memcmp`.
- Against openpilot's OpenCL interpolation on the actual `640x360` K230 source,
  the worst 12-case mean absolute pixel difference is `0.314/255` and the worst
  absolute difference is `7/255`. The paths use the same projection and YUV6
  layout, but are not bit-exact because openpilot quantizes coordinates to 1/32
  pixel with 15-bit coefficients while K230 uses 12-bit coefficients.
- Automatic calibration requires both CAN `vEgo` and camera-odometry `trans[0]`
  above 15 mph, matching openpilot's acceptance gate.
- The final graph keeps both visual towers live. `input_imgs` uses the
  910-pixel-focal medmodel virtual camera and `big_input_imgs` uses the
  455-pixel-focal sbigmodel virtual camera, matching the single-camera C2 path.

Run the host-only verifier:

```sh
cmake -S . -B /tmp/supercombo_k230_verify \
  -DCMAKE_BUILD_TYPE=Release \
  -DSUPERCOMBO_BUILD_RUNTIME=OFF \
  -DSUPERCOMBO_BUILD_BENCHMARKS=ON
cmake --build /tmp/supercombo_k230_verify \
  --target verify_calibration_equivalence bench_input_warp_overhead -j2
/tmp/supercombo_k230_verify/bin/verify_calibration_equivalence
/tmp/supercombo_k230_verify/bin/bench_input_warp_overhead 3000
```

## Host self-tests

The same benchmark build produces self-checking binaries that need no board:

| Target | Covers |
| --- | --- |
| `check_k7_control_replay` | K7 engage gates, torque limits, CAN frame build |
| `check_departure_alert` | departure alert state machine |
| `check_adaptive_cruise` | vision cruise button pacing and limits |
| `check_model_output_parser` | supercombo raw-output layout |
| `check_k230_can_queue` | shared-memory CAN queue |
| `check_panda_can_codec` | panda USB CAN packing/unpacking |

See [Diagnostics](diagnostics.md) for the build command and additional
on-board tools.
