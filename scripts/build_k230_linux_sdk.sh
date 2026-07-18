#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
documents_dir="/Users/chan/Documents"
sdk_dir="${K230_LINUX_SDK_DIR:-${documents_dir}/K230/downloads/k230_linux_sdk}"
build_dir="${repo_dir}/build/k230-linux-buildroot"
image="${K230_BUILDER_IMAGE:-ghcr.io/huangzhenming/k230-builder:latest}"
output_dir="${sdk_dir}/output/k230_canmv_01studio_defconfig"
sysroot="${output_dir}/host/riscv64-buildroot-linux-gnu/sysroot"

docker run --rm --platform linux/amd64 \
  -v k230-rtos-toolchains:/opt/toolchains \
  "${image}" download-toolchains tc2

docker run --rm --platform linux/amd64 \
  -v "${documents_dir}:${documents_dir}" \
  -v k230-rtos-toolchains:/opt/toolchains \
  -w "${repo_dir}" \
  "${image}" \
  bash -lc "
    set -euo pipefail
    export PKG_CONFIG_SYSROOT_DIR='${sysroot}'
    export PKG_CONFIG_LIBDIR='${sysroot}/usr/lib/pkgconfig:${sysroot}/usr/share/pkgconfig'
    cmake -S . -B '${build_dir}' \\
      -DCMAKE_BUILD_TYPE=Release \\
      -DCMAKE_TOOLCHAIN_FILE=toolchains/k230-linux-sdk.cmake \\
      -DK230_LINUX_SDK_DIR='${sdk_dir}' \\
      -DK230_SYSROOT='${sysroot}' \\
      -DOpenCV_DIR='${sysroot}/usr/lib/cmake/opencv4' \\
      -DSUPERCOMBO_BUILD_RUNTIME=ON \\
      -DSUPERCOMBO_BUILD_BENCHMARKS=OFF \\
      -DSUPERCOMBO_BUILD_PANDA=ON
    cmake --build '${build_dir}' --parallel \"\$(nproc)\"
  "
