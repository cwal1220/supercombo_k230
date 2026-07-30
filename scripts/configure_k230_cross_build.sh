#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${1:-build-k230-sdk}"
build_type="${CMAKE_BUILD_TYPE:-Release}"
sdk_dir="${K230_LINUX_SDK_DIR:-/Users/chan/Documents/K230/downloads/k230_linux_sdk}"
output_dir="${sdk_dir}/output/k230_canmv_01studio_defconfig"
host_dir="${output_dir}/host"
sysroot="${host_dir}/riscv64-buildroot-linux-gnu/sysroot"
xuantie_dir="${K230_XUANTIE_TOOLCHAIN_DIR:-/Users/chan/Documents/K230/.docker-toolchain/Xuantie-900-gcc-linux-6.6.0-glibc-x86_64-V3.0.2}"
image="${K230_CONFIGURE_IMAGE:-ubuntu:24.04}"

for required in \
  "${host_dir}/bin/riscv64-unknown-linux-gnu-g++" \
  "${sysroot}/usr/lib/libasound.so" \
  "${sysroot}/usr/lib/libv4l2-drm.so" \
  "${xuantie_dir}/bin/riscv64-unknown-linux-gnu-g++"; do
  if [ ! -e "${required}" ]; then
    echo "Missing K230 SDK dependency: ${required}" >&2
    exit 1
  fi
done

docker run --rm --platform linux/amd64 \
  -v "${repo_dir}:/work" \
  -v "${host_dir}:/sdk" \
  -v "${xuantie_dir}:/opt/toolchain/Xuantie-900-gcc-linux-6.6.0-glibc-x86_64-V3.0.2" \
  -w /work \
  "${image}" \
  bash -lc '
    set -euo pipefail
    build_dir="$1"
    build_type="$2"
    sysroot=/sdk/riscv64-buildroot-linux-gnu/sysroot
    export PATH=/sdk/bin:$PATH
    export PKG_CONFIG_SYSROOT_DIR="$sysroot"
    export PKG_CONFIG_LIBDIR="$sysroot/usr/lib/pkgconfig:$sysroot/usr/share/pkgconfig"
    cmake -S . -B "$build_dir" -G Ninja \
      -DCMAKE_BUILD_TYPE="$build_type" \
      -DCMAKE_SYSTEM_NAME=Linux \
      -DCMAKE_SYSTEM_PROCESSOR=riscv64 \
      -DCMAKE_TRY_COMPILE_TARGET_TYPE=STATIC_LIBRARY \
      -DCMAKE_C_COMPILER=/sdk/bin/riscv64-unknown-linux-gnu-gcc \
      -DCMAKE_CXX_COMPILER=/sdk/bin/riscv64-unknown-linux-gnu-g++ \
      -DCMAKE_SYSROOT="$sysroot" \
      -DCMAKE_C_FLAGS="-mcpu=c908v -mabi=lp64d" \
      -DCMAKE_CXX_FLAGS="-mcpu=c908v -mabi=lp64d" \
      -DCMAKE_FIND_ROOT_PATH="$sysroot" \
      -DK230_SYSROOT="$sysroot" \
      -DOpenCV_DIR="$sysroot/usr/lib/cmake/opencv4" \
      -DSUPERCOMBO_BUILD_RUNTIME=ON \
      -DSUPERCOMBO_BUILD_BENCHMARKS=OFF \
      -DSUPERCOMBO_BUILD_PANDA=ON
  ' bash "${build_dir}" "${build_type}"
