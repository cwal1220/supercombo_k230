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

## Upload to the board

Upload the rebuilt runtime files:

```sh
K230_SSH="sshpass -p '<password>' ssh" \
K230_SCP="sshpass -p '<password>' scp" \
  scripts/upload_to_board.sh root@192.168.219.111
```

The upload script reads binaries from `build/bin` by default. Set
`K230_BUILD_DIR=build-native` for an on-board build or `K230_BIN_DIR` for a
custom binary directory.

Runtime tuning and calibration JSON files already present under `params/` are
never overwritten by the upload script. Repository defaults are copied to
`params.defaults/` and seed a runtime file only when that file does not exist.

All CMake executables are written below the selected build directory in `bin/`.
Build directories are local generated output and are not tracked:

| Directory | Build |
| --- | --- |
| `build/` | macOS SDK cross-build, the upload default |
| `build-native/` | on-board native build |
| `build-host/` | host self-checks (`scripts/run_host_checks.sh`) |

> [!WARNING]
> Do not use a generic Ubuntu riscv64 compiler for board binaries: it can link
> against a newer glibc than the flashed K230 image provides. The Xuantie
> sysroot used by `configure_k230_macos.sh` is glibc 2.33, matching the board.
