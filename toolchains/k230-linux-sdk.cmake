set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR riscv64)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(K230_LINUX_SDK_DIR
    "/Users/chan/Documents/K230/downloads/k230_linux_sdk"
    CACHE PATH "K230 Linux SDK directory")
set(K230_BUILDROOT_OUTPUT
    "${K230_LINUX_SDK_DIR}/output/k230_canmv_01studio_defconfig"
    CACHE PATH "K230 Buildroot output directory")
set(K230_HOST_DIR "${K230_BUILDROOT_OUTPUT}/host" CACHE PATH
    "K230 Buildroot host tools directory")
set(K230_SYSROOT
    "${K230_HOST_DIR}/riscv64-buildroot-linux-gnu/sysroot"
    CACHE PATH "K230 target sysroot")

set(CMAKE_C_COMPILER
    "${K230_HOST_DIR}/bin/riscv64-unknown-linux-gnu-gcc")
set(CMAKE_CXX_COMPILER
    "${K230_HOST_DIR}/bin/riscv64-unknown-linux-gnu-g++")
set(CMAKE_SYSROOT "${K230_SYSROOT}")

set(CMAKE_C_FLAGS_INIT "-mcpu=c908v -mabi=lp64d")
set(CMAKE_CXX_FLAGS_INIT "-mcpu=c908v -mabi=lp64d")

set(CMAKE_FIND_ROOT_PATH "${K230_SYSROOT}")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
