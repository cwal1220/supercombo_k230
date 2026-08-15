# Build and deploy

[← Documentation index](../README.md)

## Native board build

```sh
cd /root/supercombo_k230
./scripts/fetch_nncase_runtime.sh
cmake -S . -B build-native \
  -DCMAKE_BUILD_TYPE=Release \
  -DSUPERCOMBO_BUILD_PANDA=ON
cmake --build build-native -j2
cmake --install build-native --prefix /root/supercombo_k230
./k230_manager.py
```

The Panda build is required by the manager's default full-pipeline mode. For a
camera/model/display-only build, omit `SUPERCOMBO_BUILD_PANDA` and start the
manager with `K230_ENABLE_CONTROL=0 K230_ENABLE_PANDA=0`.

## macOS SDK cross-build

For the Homebrew-based macOS host setup used by this workspace, see
[macOS K230 cross-build environment](macos-build-environment.md). The short form
is:

```sh
cd /path/to/k230/supercombo_k230
./scripts/configure_k230_macos.sh
cd build
cmake ..
make -j2
```

The macOS cross-build defaults to the real-vehicle pipeline, including
`k230_pandad` and `k230_controlsd`. Set `SUPERCOMBO_BUILD_PANDA=OFF` only for
camera/model/display-only builds.

## Docker/Buildroot SDK cross-build

The Docker/Buildroot SDK flow remains available when a Buildroot SDK is required
for a different target image.

The repository can also be cross-built from macOS using the Buildroot host tools
produced by `k230_linux_sdk`. This is required to keep the target glibc ABI at or
below the board's glibc 2.33. First fetch the nncase runtime deps and build the
`k230_canmv_01studio_defconfig` SDK output:

```sh
cd /path/to/k230/supercombo_k230
./scripts/fetch_nncase_runtime.sh
```

Then configure and build with the SDK host wrapper and Xuantie toolchain. The
script defaults to the SDK and toolchain paths under
`/Users/chan/Documents/K230`; override `K230_LINUX_SDK_DIR` or
`K230_XUANTIE_TOOLCHAIN_DIR` when needed:

```sh
scripts/configure_k230_cross_build.sh

docker run --rm --platform linux/amd64 \
  -v "$PWD":/work \
  -v /Users/chan/Documents/K230/downloads/k230_linux_sdk/output/k230_canmv_01studio_defconfig/host:/sdk \
  -v /Users/chan/Documents/K230/.docker-toolchain/Xuantie-900-gcc-linux-6.6.0-glibc-x86_64-V3.0.2:/opt/toolchain/Xuantie-900-gcc-linux-6.6.0-glibc-x86_64-V3.0.2 \
  -w /work \
  ubuntu:24.04 \
  bash -lc 'export PATH=/sdk/bin:$PATH; cmake --build build-k230-sdk -j$(nproc)'
```

## Upload to the board

Upload the rebuilt runtime files:

```sh
K230_SSH="sshpass -p '<password>' ssh" \
K230_SCP="sshpass -p '<password>' scp" \
  scripts/upload_to_board.sh root@192.168.219.111
```

The upload script reads binaries from `build-k230-sdk/bin` by default. Set
`K230_BUILD_DIR=build-native` for an on-board build or `K230_BIN_DIR` for a
custom binary directory.

Runtime tuning and calibration JSON files already present under `params/` are
never overwritten by the upload script. Repository defaults are copied to
`params.defaults/` and seed a runtime file only when that file does not exist.

All CMake executables are written below the selected build directory in `bin/`.
`build-k230-sdk/` is local generated output and is not tracked.

> [!WARNING]
> Do not use a generic Ubuntu riscv64 compiler for board binaries: it can link
> against a newer glibc than the flashed K230 image provides.
