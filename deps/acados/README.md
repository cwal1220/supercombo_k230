# K230 Acados runtime

This directory contains the RISC-V Acados 0.1.8 runtime and generated lateral
MPC solver used by the K230 controller. The headers, shared libraries, and
`acados_solver_lat.h` were taken from the validated
`openpilot_c2_k230` K230 build. They target riscv64 glibc and are loaded from
the application's `lib/` directory on the board.

The corresponding license texts are included as `LICENSE.acados`,
`LICENSE.blasfeo`, `LICENSE.hpipm`, and `LICENSE.qpOASES`.
