#!/usr/bin/env bash
set -euo pipefail

BUILD_DIR="${1:-build-mac-rv64}"
IMAGE="${K230_TOOLCHAIN_IMAGE:-supercombo-k230-toolchain:24.04}"
BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"

if [ ! -d deps/k230_sysroot/usr/lib/riscv64-linux-gnu ]; then
  echo "Missing deps/k230_sysroot. Run scripts/sync_k230_sysroot.sh first." >&2
  exit 1
fi

docker run --rm --platform linux/arm64/v8 \
  -v "$PWD":/work \
  -w /work \
  "$IMAGE" \
  bash -lc "PKG_CONFIG_SYSROOT_DIR=/work/deps/k230_sysroot \
    PKG_CONFIG_LIBDIR=/work/deps/k230_sysroot/usr/lib/riscv64-linux-gnu/pkgconfig \
    cmake -S . -B $BUILD_DIR \
      -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
      -DCMAKE_C_COMPILER=riscv64-linux-gnu-gcc \
      -DCMAKE_CXX_COMPILER=riscv64-linux-gnu-g++ \
      -DSUPERCOMBO_BUILD_PANDA=ON \
      -DK230_SYSROOT_LIB_DIR=/work/deps/k230_sysroot/usr/lib/riscv64-linux-gnu \
      -DOpenCV_DIR=/work/deps/k230_sysroot/usr/lib/riscv64-linux-gnu/cmake/opencv4"
