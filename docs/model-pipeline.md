# Model pipeline

[← Documentation index](../README.md)

The model path captures the AI stream as `NV12 640x360` through `/dev/video2`
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

The source intrinsics are scaled from the calibrated K230 camera matrix, so the
default `640x360` path uses `fx=541.91`, `fy=528.66`, `cx=315.38`, and
`cy=179.11`. `SUPERCOMBO_INPUT_WARP_FX/FY/CX/CY` can override these values for a
separately measured camera pipeline.

The medmodel transform feeds the current and previous frames into `input_imgs`;
the wider C2 sbigmodel transform independently feeds its current and previous
frames into `big_input_imgs`.

## Model output

The supercombo graph emits 6012 floats plus a 512-float recurrent state that is
fed back on the next frame. `src/model_output.*` owns the layout:

| Block | Offset | Floats | Contents |
| --- | ---: | ---: | --- |
| plan | 0 | 4955 | 5 hypotheses x (33 points x 15 values, mean/std) + probability |
| lane lines | 4955 | 528 | 4 lines x 33 points, mean/std |
| lane probabilities | 5483 | 8 | per-line existence |
| road edges | 5491 | 264 | 2 edges x 33 points, mean/std |
| leads | 5755 | 102 | 2 hypotheses x 6 timesteps x (x, y, v, a) |
| lead probabilities | 5857 | 3 | 0/2/4 s selection |
| stop line | 5860 | 52 | 3 hypotheses x (position, rotation, speed, time) |
| meta/desire | 5912 | 8 | desire softmax |
| pose | 6000 | 12 | translation/rotation and standard deviations |
| recurrent | 6012 | 512 | fed back into the next inference |

Of the 15 values per plan point, the parser consumes position (0–2),
orientation (9–11), and orientation rate (12–14).
