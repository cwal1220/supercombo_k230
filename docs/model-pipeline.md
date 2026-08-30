# Model pipeline

[← Documentation index](../README.md)

The model path captures the AI stream as `NV12 1280x720` through `/dev/video2`
crop/resize, samples independent `512x256` medmodel and sbigmodel views from the
same source frame, prepares both YUV6 recurrent inputs, runs nncase runtime
directly, and publishes compact `modelState`. The overlay display process uses
`/dev/video1` for preview and `/dev/video2` remains dedicated to the AI stream.

## Input transform

The model input preparation always uses calibrated homography sampling fused
with `NV12 -> YUV6` conversion. Compact fixed-point lookup tables and a C908 RVV
indexed-gather kernel write the current frame directly into the second half of
each nncase image tensor; after inference, that half is copied to the
previous-frame half. The shared camera frame is copied once into a cacheable
buffer because C908 `vluxei32.v` is not reliable on the `/dev/shm` ring mapping.

No warped image, YUV6 staging tensor, or full `[previous,current]` pack buffer is
created.

## Intrinsics

The source intrinsics are scaled from the measured `1920x1080` K230 camera
matrix in `assets/calibration/intrinsics_20260822.json`, so the default
`1280x720` path uses `fx=1055.60`, `fy=1055.84`, `cx=636.63`, and `cy=363.45`.
`SUPERCOMBO_INPUT_WARP_FX/FY/CX/CY` can override these values for a separately
measured camera pipeline.

That matrix comes from a 23-view checkerboard solve (0.36 px reprojection RMS,
bootstrap sigma 4.4 px on `fx`). It replaces an earlier matrix whose `fx/fy`
ratio was 1.025; this sensor has square pixels, so the 2.5% anisotropy was
measurement error rather than a real focal difference, and it was almost all in
`fx` (-2.60%, against -0.14% for `fy`). The warp is a pinhole homography like
openpilot's, so the measured lens distortion coefficients are recorded in the
JSON but not applied.

The medmodel transform feeds the current and previous frames into `input_imgs`;
the wider sbigmodel transform independently feeds its current and previous
frames into `big_input_imgs`.

## Model output

The supercombo graph (openpilot v0.9.4) emits 6120 floats, the last 128 of
which (before a 2-float pad) are the feature vector fed back through the
99-tick feature buffer. `src/model_output.*` owns the layout:

| Block | Offset | Floats | Contents |
| --- | ---: | ---: | --- |
| plan | 0 | 4955 | 5 hypotheses x (33 points x 15 values, mean/std) + probability |
| lane lines | 4955 | 528 | 4 lines x 33 points, mean/std |
| lane probabilities | 5483 | 8 | per-line existence |
| road edges | 5491 | 264 | 2 edges x 33 points, mean/std |
| leads | 5755 | 102 | 2 hypotheses x 6 timesteps x (x, y, v, a) |
| lead probabilities | 5857 | 3 | 0/2/4 s selection |
| meta/desire | 5860 | 8 | desire softmax |
| pose | 5948 | 12 | translation/rotation and standard deviations |
| wide_from_device_euler | 5960 | 6 | unused by this runtime |
| sim_pose | 5966 | 12 | unused by this runtime |
| road_transform | 5978 | 12 | unused by this runtime |
| feature | 5990 | 128 | fed back into the next inference |

Of the 15 values per plan point, the parser consumes position (0–2),
orientation (9–11), and orientation rate (12–14).
