# macOS K230 cross-build environment

This document describes the supported macOS host setup for the K230 runtime
build. Common host tools are installed once with Homebrew. The Xuantie target
toolchain, K230 sysroot, board libraries, and RISC-V linker remain workspace
assets because they are target-specific and are not interchangeable with the
macOS versions of those tools.

## 1. Install common host tools

Install Homebrew first if it is not already available, then run:

```sh
brew install cmake llvm pkg-config binutils z3 zstd
```

The build uses these Homebrew tools:

- `cmake` for configuration
- `llvm` for the Clang cross compiler and LLVM binutils
- macOS Command Line Tools `make` for the Unix Makefiles generator
- `pkg-config` for host-side package discovery
- `z3` and `zstd` for LLVM runtime dependencies on macOS

Homebrew `binutils` provides general host utilities. It does not replace the
RISC-V linker used by this project; the project keeps a target-specific GNU
linker at `host-tools/binutils-build-riscv/ld/ld-new`.

## 2. Required workspace layout

The configuration script expects this layout when run from the standard
workspace:

```text
k230/
├── supercombo_k230/                # this Git repository
│   └── build/                      # generated build output
├── toolchain/xuantie-900/          # Xuantie compiler and target sysroot
├── host-tools/binutils-build-riscv/ld/ld-new
├── third-party/board-libs/usr/lib/
├── third-party/drm-dev/usr/include/
├── third-party/opencv/cmake/
└── tools/target-pkg-config
```

The nncase runtime libraries are tracked under `supercombo_k230/deps/`.

If the workspace is elsewhere, set `K230_WORKSPACE_DIR` or override the
individual `K230_*` variables used by the configuration script.

## 3. Configure and build

On the first run, configure the in-tree `build/` directory from the repository
directory:

```sh
cd /path/to/k230/supercombo_k230
./scripts/configure_k230_macos.sh
```

After that, the normal incremental workflow is:

```sh
cd /path/to/k230/supercombo_k230/build
cmake ..
make -j2
```

The script automatically uses the Homebrew prefixes for CMake and LLVM and the
system `make` command. It validates the target-specific paths before
configuring. The default runtime build produces:

```text
build/bin/k230_camerad
build/bin/k230_modeld
build/bin/k230_overlayd
build/bin/k230_recordd
build/bin/k230_pandad
build/bin/k230_controlsd
```

The default is the real-vehicle build: Panda USB/CAN bridging and the lateral
control process are enabled. Use `SUPERCOMBO_BUILD_PANDA=OFF` only for a
camera/model/display-only build.

To use the standard workspace path explicitly:

```sh
export K230_WORKSPACE_DIR=/Users/chan/Documents/k230
cd "$K230_WORKSPACE_DIR/supercombo_k230"
./scripts/configure_k230_macos.sh
cd build
cmake ..
make -j2
```

## 4. Useful overrides

```sh
K230_XUANTIE_TOOLCHAIN_DIR=/path/to/xuantie-900 \
K230_RISCV_LD=/path/to/ld-new \
./scripts/configure_k230_macos.sh /path/to/build
```

Set `SUPERCOMBO_BUILD_PANDA=OFF` when the target-side Panda/libusb dependencies
are intentionally excluded. `SUPERCOMBO_BUILD_BENCHMARKS=ON` can be used to
add benchmark targets without changing the runtime pipeline.
