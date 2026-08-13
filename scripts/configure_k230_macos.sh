#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_dir="${K230_WORKSPACE_DIR:-$(cd "${repo_dir}/.." && pwd)}"
build_dir="${1:-${repo_dir}/build}"
build_type="${CMAKE_BUILD_TYPE:-Release}"

cmake_bin="${CMAKE_BIN:-$(brew --prefix cmake)/bin/cmake}"
llvm_dir="${LLVM_ROOT:-$(brew --prefix llvm)}"
make_bin="${MAKE_BIN:-$(command -v make)}"

toolchain_dir="${K230_XUANTIE_TOOLCHAIN_DIR:-${workspace_dir}/toolchain/xuantie-900}"
sysroot="${K230_SYSROOT:-${toolchain_dir}/sysroot}"
board_libs_dir="${K230_BOARD_LIBS_DIR:-${workspace_dir}/third-party/board-libs}"
drm_dir="${K230_DRM_DIR:-${workspace_dir}/third-party/drm-dev}"
opencv_dir="${K230_OPENCV_DIR:-${workspace_dir}/third-party/opencv}"
nncase_dir="${NNCASE_DEPS_DIR:-${repo_dir}/deps}"
pkg_config="${PKG_CONFIG_EXECUTABLE:-${repo_dir}/tools/target-pkg-config}"
riscv_ld="${K230_RISCV_LD:-${workspace_dir}/host-tools/binutils-build-riscv/ld/ld-new}"
panda_build="${SUPERCOMBO_BUILD_PANDA:-ON}"
benchmarks_build="${SUPERCOMBO_BUILD_BENCHMARKS:-OFF}"

gcc_lib_dir="${toolchain_dir}/lib/gcc/riscv64-unknown-linux-gnu/14.1.1/lib64xthead/lp64d"
cxx_include_dir="${toolchain_dir}/riscv64-unknown-linux-gnu/include/c++/14.1.1"
cxx_target_include_dir="${cxx_include_dir}/riscv64-unknown-linux-gnu/lib64xthead/lp64d"
sysroot_base_lib_dir="${sysroot}/lib64xthead/lp64d"
sysroot_usr_lib_dir="${sysroot}/usr/lib64xthead/lp64d"
board_lib_dir="${board_libs_dir}/usr/lib"
drm_include_dir="${drm_dir}/usr/include"

for required in \
  "${cmake_bin}" \
  "${make_bin}" \
  "${llvm_dir}/bin/clang" \
  "${llvm_dir}/bin/clang++" \
  "${llvm_dir}/bin/llvm-ar" \
  "${llvm_dir}/bin/llvm-ranlib" \
  "${toolchain_dir}/riscv64-unknown-linux-gnu/bin/ld" \
  "${sysroot}" \
  "${board_lib_dir}" \
  "${drm_include_dir}" \
  "${opencv_dir}/cmake" \
  "${nncase_dir}" \
  "${pkg_config}" \
  "${riscv_ld}"; do
  if [ ! -e "${required}" ]; then
    echo "Missing K230 build dependency: ${required}" >&2
    exit 1
  fi
done

cmake_flags="-march=rv64gcv -mabi=lp64d -B${gcc_lib_dir} -I${drm_include_dir}"
cxx_flags="${cmake_flags} -nostdinc++ -isystem ${cxx_include_dir} -isystem ${cxx_target_include_dir} -stdlib=libstdc++"
linker_flags="-B${gcc_lib_dir} -fuse-ld=${riscv_ld} -rtlib=libgcc -unwindlib=libgcc"
linker_flags+=" -Wl,--dynamic-linker,/lib/ld-linux-riscv64-lp64d.so.1 -Wl,--no-relax"
linker_flags+=" -L${gcc_lib_dir} -L${toolchain_dir}/riscv64-unknown-linux-gnu/lib64xthead/lp64d"
linker_flags+=" -L${sysroot_base_lib_dir} -L${sysroot_usr_lib_dir} -L${board_lib_dir}"
linker_flags+=" -Wl,-rpath-link,${gcc_lib_dir}"
linker_flags+=" -Wl,-rpath-link,${toolchain_dir}/riscv64-unknown-linux-gnu/lib64xthead/lp64d"
linker_flags+=" -Wl,-rpath-link,${sysroot_base_lib_dir} -Wl,-rpath-link,${sysroot_usr_lib_dir}"
linker_flags+=" -Wl,-rpath-link,${board_lib_dir}"

mkdir -p "${build_dir}"
"${cmake_bin}" --fresh \
  -S "${repo_dir}" \
  -B "${build_dir}" \
  -G "Unix Makefiles" \
  -DCMAKE_BUILD_TYPE="${build_type}" \
  -DCMAKE_SYSTEM_NAME=Linux \
  -DCMAKE_SYSTEM_PROCESSOR=riscv64 \
  -DCMAKE_C_COMPILER="${llvm_dir}/bin/clang" \
  -DCMAKE_CXX_COMPILER="${llvm_dir}/bin/clang++" \
  -DCMAKE_C_COMPILER_TARGET=riscv64-unknown-linux-gnu \
  -DCMAKE_CXX_COMPILER_TARGET=riscv64-unknown-linux-gnu \
  -DCMAKE_AR="${llvm_dir}/bin/llvm-ar" \
  -DCMAKE_RANLIB="${llvm_dir}/bin/llvm-ranlib" \
  -DCMAKE_NM="${llvm_dir}/bin/llvm-nm" \
  -DCMAKE_OBJCOPY="${llvm_dir}/bin/llvm-objcopy" \
  -DCMAKE_OBJDUMP="${llvm_dir}/bin/llvm-objdump" \
  -DCMAKE_READELF="${llvm_dir}/bin/llvm-readelf" \
  -DCMAKE_STRIP="${llvm_dir}/bin/llvm-strip" \
  -DCMAKE_TRY_COMPILE_TARGET_TYPE=STATIC_LIBRARY \
  -DCMAKE_SYSROOT="${sysroot}" \
  -DCMAKE_C_FLAGS="${cmake_flags}" \
  -DCMAKE_CXX_FLAGS="${cxx_flags}" \
  -DCMAKE_EXE_LINKER_FLAGS="${linker_flags}" \
  -DCMAKE_SKIP_RPATH=ON \
  -DK230_SYSROOT="${sysroot}" \
  -DK230_SYSROOT_BASE_LIB_DIR="${sysroot_base_lib_dir}" \
  -DK230_SYSROOT_INCLUDE_DIR="${drm_include_dir}" \
  -DK230_SYSROOT_LIB_DIR="${board_lib_dir}" \
  -DNNCASE_DEPS_DIR="${nncase_dir}" \
  -DOpenCV_DIR="${opencv_dir}/cmake" \
  -DPKG_CONFIG_EXECUTABLE="${pkg_config}" \
  -DSUPERCOMBO_BUILD_RUNTIME=ON \
  -DSUPERCOMBO_BUILD_PANDA="${panda_build}" \
  -DSUPERCOMBO_BUILD_BENCHMARKS="${benchmarks_build}"
