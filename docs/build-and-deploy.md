# Build and deploy

[← Documentation index](../README.md)

All CMake executables are written to `bin/` below the build directory. Build
directories are generated output and are not tracked:

| Directory | Build |
| --- | --- |
| `build/` | macOS cross-build, the upload default |
| `build-native/` | on-board native build |
| `build-host/` | host tests and tools (`scripts/run_host_tests.sh`) |

The default is the real-vehicle build: the Panda USB/CAN bridge and
`k230_controlsd` are included. `-DSUPERCOMBO_BUILD_PANDA=OFF` builds the
camera/model/display processes only; start the manager with
`K230_ENABLE_CONTROL=0 K230_ENABLE_PANDA=0` in that case.
`-DSUPERCOMBO_BUILD_DIAGNOSTICS=ON` adds the tools in
[diagnostics/](../diagnostics/README.md) without changing the runtime.

Every board build needs `deps/`, which is not tracked.
`scripts/fetch_nncase_runtime.sh` recreates it from the Kendryte nncase v2.11.0
release plus gsl-lite 0.37.0, both pinned by SHA256:

- `deps/include/nncase/runtime/interpreter.h`
- `deps/include/gsl/gsl-lite.hpp` (27 nncase headers include it; the nncase
  tarball does not ship it)
- `deps/lib/libNncase.Runtime.Native.a`
- `deps/lib/libnncase.rt_modules.k230.a`
- `deps/lib/libfunctional_k230.a`

## macOS cross-build

Common host tools come from Homebrew. The Xuantie target toolchain, K230
sysroot, board libraries, and RISC-V linker stay in the workspace because they
are target-specific and not interchangeable with their macOS counterparts.

```sh
brew install cmake llvm pkg-config binutils z3 zstd
```

- `cmake` configures, and the macOS Command Line Tools `make` builds
- `llvm` provides the Clang cross compiler and LLVM binutils
- `pkg-config` does host-side package discovery
- `z3` and `zstd` are LLVM runtime dependencies on macOS
- Homebrew `binutils` provides general host utilities; it does not replace the
  target GNU linker at `host-tools/binutils-build-riscv/ld/ld-new`

`scripts/configure_k230_macos.sh` expects this workspace layout:

```text
k230/
├── supercombo_k230/                # this repository (tools/target-pkg-config lives here)
│   └── build/                      # generated build output
├── toolchain/xuantie-900/          # Xuantie compiler and target sysroot
├── host-tools/binutils-build-riscv/ld/ld-new
├── third-party/board-libs/usr/lib/
├── third-party/drm-dev/usr/include/
└── third-party/opencv/cmake/
```

If the workspace is elsewhere, set `K230_WORKSPACE_DIR`, or override the
individual paths (`K230_XUANTIE_TOOLCHAIN_DIR`, `K230_RISCV_LD`, ...). The script
validates them before configuring. Configure once, then build incrementally:

```sh
cd /path/to/k230/supercombo_k230
./scripts/configure_k230_macos.sh          # optional argument: build directory
cd build
cmake ..
make -j2
```

The runtime build produces `k230_camerad`, `k230_modeld`, `k230_overlayd`,
`k230_recordd`, `k230_pandad`, and `k230_controlsd` in `build/bin/`.

> [!WARNING]
> Do not use a generic Ubuntu riscv64 compiler for board binaries: it can link
> against a newer glibc than the flashed K230 image provides. The Xuantie
> sysroot used by `configure_k230_macos.sh` is glibc 2.33, matching the board.

## Native board build

After the packages in [Board setup](board-setup.md) are installed:

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

The board GCC assembler does not accept the T-Head mnemonic `dcache.civa`, so
`include/thead.h` and `src/mmz.c` use the equivalent raw instruction:

```c
__asm volatile(".insn i 0x0b, 0, x0, %0, 0x027" : : "r"(op_addr));
```

Keep that form unless the assembler supports the `xtheadcmo` extension
mnemonic.

## Upload to the board

```sh
K230_SSH="sshpass -p '<password>' ssh" \
K230_SCP="sshpass -p '<password>' scp" \
  scripts/upload_to_board.sh root@192.168.219.111
```

The upload script reads binaries from `build/bin` by default. Set
`K230_BUILD_DIR=build-native` for an on-board build or `K230_BIN_DIR` for a
custom binary directory.

Runtime tuning and calibration JSON files already present under `params/` are
never overwritten. Repository defaults are copied to `params.defaults/` and seed
a runtime file only when that file does not exist.
