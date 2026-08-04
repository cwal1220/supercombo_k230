# K230 modern driving model

`supercombo.kmodel` is the selected six-input `driving_supercombo` artifact for
the K230 runtime on `feat/modern-openpilot-k230`.

## Artifact

```text
908ec08594776d0060e26dbd7adca68831dc88433a175940a7fe89cce30c151d  supercombo.kmodel
```

It was built from sunnypilot's `driving_supercombo.onnx` at commit
`1a07e4722853c0606b0e1caa8f300a371e342948`. The source ONNX SHA-256 is
`659727c4d4839adc4992a254409a54259a8756a743f2d567bf5fdc6579f8009b`.

The Kmodel uses nncase 2.11.0, 288 real driving calibration samples, KLD
calibration, uint8 weights and a selected mixed uint8/int16 activation scheme.
Its matching 2576-channel output affine is embedded in the graph.

## ABI

```text
0 img                 uint8   [1, 12, 128, 256]
1 big_img             uint8   [1, 12, 128, 256]
2 features_buffer     float16 [1, 24, 512]
3 desire_pulse        float16 [1, 25, 8]
4 traffic_convention  float16 [1, 2]
5 action_t            float16 [1, 2]

0 outputs             float32 [1, 2576]
```

This artifact is not compatible with the previous five-input GRU runtime. The
runtime performs an exact ABI check before inference and implements the modern
four-frame image cadence, hidden-feature queue and desire max pooling.

The model has no stop-line output. The parser marks that field unavailable.

## Evidence

- `modern_model_metadata.json` records provenance, compiler settings and ABI.
- `verification/modern_model_board_validation.json` records host, replay,
  live-camera and quality checks.
- `../docs/modern_model_migration.md` records the rollout and rollback gates.

The final board measurements are 22.48 FPS for a 1,200-frame full-runtime
replay and 20.003 FPS steady cadence for a 1,200-frame live-camera run, both
with zero model errors.

Legacy ONNX/PTQ files and historical verification JSONs remain for audit only.
Do not run the old `Gemm/Split`, `Elu_223` or GRU rewrite path against this
modern model.
