#!/usr/bin/env bash
set -euo pipefail

BOARD="${1:-root@192.168.219.115}"
SYSROOT="${K230_SYSROOT_DIR:-deps/k230_sysroot}"
RSH="${K230_RSYNC_RSH:-ssh -o PubkeyAuthentication=no -o PreferredAuthentications=password -o StrictHostKeyChecking=no}"

mkdir -p \
  "$SYSROOT/usr/include" \
  "$SYSROOT/usr/lib/riscv64-linux-gnu" \
  "$SYSROOT/usr/lib/riscv64-linux-gnu/pkgconfig" \
  "$SYSROOT/lib/riscv64-linux-gnu"

rsync_remote() {
  local remote_path="$1"
  local local_dir="$2"
  rsync -a -e "$RSH" "$BOARD:$remote_path" "$local_dir"
}

rsync_remote "/usr/include/opencv4" "$SYSROOT/usr/include/"
rsync_remote "/usr/include/libusb-1.0" "$SYSROOT/usr/include/"
rsync_remote "/usr/lib/riscv64-linux-gnu/cmake" "$SYSROOT/usr/lib/riscv64-linux-gnu/"
rsync_remote "/usr/lib/riscv64-linux-gnu/pkgconfig/libusb-1.0.pc" "$SYSROOT/usr/lib/riscv64-linux-gnu/pkgconfig/"

for pattern in \
  "/usr/lib/riscv64-linux-gnu/libopencv*.so*" \
  "/usr/lib/riscv64-linux-gnu/libusb-1.0.so*" \
  "/usr/lib/riscv64-linux-gnu/libv4l2-drm.so*" \
  "/usr/lib/riscv64-linux-gnu/libdisplay.so*" \
  "/usr/lib/riscv64-linux-gnu/libdrm.so*"; do
  rsync_remote "$pattern" "$SYSROOT/usr/lib/riscv64-linux-gnu/"
done

for pattern in \
  "/lib/riscv64-linux-gnu/libz.so*" \
  "/lib/riscv64-linux-gnu/libGLX.so*" \
  "/lib/riscv64-linux-gnu/libGLdispatch.so*" \
  "/lib/riscv64-linux-gnu/libX11.so*" \
  "/lib/riscv64-linux-gnu/libxcb.so*" \
  "/lib/riscv64-linux-gnu/libXau.so*" \
  "/lib/riscv64-linux-gnu/libXdmcp.so*" \
  "/lib/riscv64-linux-gnu/libbsd.so*" \
  "/lib/riscv64-linux-gnu/libmd.so*" \
  "/lib/riscv64-linux-gnu/libgfortran.so*" \
  "/lib/riscv64-linux-gnu/libgomp.so*" \
  "/lib/riscv64-linux-gnu/libcap.so*" \
  "/lib/riscv64-linux-gnu/libudev.so*"; do
  rsync_remote "$pattern" "$SYSROOT/lib/riscv64-linux-gnu/" || true
done

opencv_lib_dir="$SYSROOT/usr/lib/riscv64-linux-gnu"
for module in core imgproc; do
  if [ -e "$opencv_lib_dir/libopencv_${module}.so.410" ]; then
    rm -f \
      "$opencv_lib_dir/libopencv_${module}.so" \
      "$opencv_lib_dir/libopencv_${module}.so.406" \
      "$opencv_lib_dir/libopencv_${module}.so.4.6.0"
    ln -s "libopencv_${module}.so.410" "$opencv_lib_dir/libopencv_${module}.so"
    ln -s "libopencv_${module}.so.410" "$opencv_lib_dir/libopencv_${module}.so.406"
    ln -s "libopencv_${module}.so.410" "$opencv_lib_dir/libopencv_${module}.so.4.6.0"
  fi
done

echo "K230 sysroot synced to $SYSROOT"
