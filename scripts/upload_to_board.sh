#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

BOARD="${1:-root@192.168.219.111}"
DEST="${K230_BOARD_DIR:-/root/supercombo_k230}"
BUILD_DIR="${K230_BUILD_DIR:-build-k230-sdk}"
BIN_DIR="${K230_BIN_DIR:-${BUILD_DIR}/bin}"
read -r -a SSH_CMD <<< "${K230_SSH:-ssh}"
read -r -a SCP_CMD <<< "${K230_SCP:-scp}"
SSH_OPTIONS=(
  -o StrictHostKeyChecking=no
)

runtime_files=(
  "${BIN_DIR}/k230_camerad"
  "${BIN_DIR}/k230_modeld"
  "${BIN_DIR}/k230_overlayd"
  "${BIN_DIR}/k230_recordd"
  scripts/k230_manager.py
  scripts/param_server.py
  scripts/k230_display_control.py
  scripts/requirements-param-server.txt
)
model="models/supercombo.kmodel"
ui_assets=(
  assets/ui/traffic_wait_red_retro-270x155-v3.png
  assets/ui/traffic_go_green_retro-270x155-v3.png
)

if [ -x "${BIN_DIR}/k230_pandad" ]; then
  runtime_files+=("${BIN_DIR}/k230_pandad" "${BIN_DIR}/k230_controlsd")
fi

for runtime_file in "${runtime_files[@]}"; do
  if [ ! -f "${runtime_file}" ]; then
    echo "Missing runtime file: ${runtime_file}" >&2
    exit 1
  fi
done
for ui_asset in "${ui_assets[@]}"; do
  if [ ! -f "${ui_asset}" ]; then
    echo "Missing UI asset: ${ui_asset}" >&2
    exit 1
  fi
done

"${SSH_CMD[@]}" "${SSH_OPTIONS[@]}" "$BOARD" \
  "test -x /etc/init.d/S35supercombo_k230 || { echo 'Missing image-provided /etc/init.d/S35supercombo_k230' >&2; exit 1; }; rm -f /etc/init.d/S95supercombo_k230; rm -rf '$DEST/.upload'; mkdir -p '$DEST/.upload' '$DEST/model' '$DEST/params' '$DEST/params.defaults'"
"${SCP_CMD[@]}" "${SSH_OPTIONS[@]}" "${runtime_files[@]}" "$BOARD:$DEST/.upload/"
"${SSH_CMD[@]}" "${SSH_OPTIONS[@]}" "$BOARD" "for source in '$DEST/.upload/'*; do mv \"\$source\" '$DEST/'; done"
"${SSH_CMD[@]}" "${SSH_OPTIONS[@]}" "$BOARD" "mkdir -p '$DEST/.upload/assets/ui' '$DEST/assets/ui'"
"${SCP_CMD[@]}" "${SSH_OPTIONS[@]}" "${ui_assets[@]}" "$BOARD:$DEST/.upload/assets/ui/"
"${SSH_CMD[@]}" "${SSH_OPTIONS[@]}" "$BOARD" \
  "rm -f '$DEST/assets/ui/'*.png; for source in '$DEST/.upload/assets/ui/'*; do mv \"\$source\" '$DEST/assets/ui/'; done"
if [ -x "${BIN_DIR}/k230_controlsd" ]; then
  "${SSH_CMD[@]}" "${SSH_OPTIONS[@]}" "$BOARD" "mkdir -p '$DEST/lib' '$DEST/.upload/lib'"
  "${SCP_CMD[@]}" "${SSH_OPTIONS[@]}" \
    deps/acados/lateral_solver/libacados_ocp_solver_lat.so \
    deps/acados/riscv64/lib/libacados.so \
    deps/acados/riscv64/lib/libblasfeo.so \
    deps/acados/riscv64/lib/libhpipm.so \
    deps/acados/riscv64/lib/libqpOASES_e.so.3.1 \
    "$BOARD:$DEST/.upload/lib/"
  "${SSH_CMD[@]}" "${SSH_OPTIONS[@]}" "$BOARD" "for source in '$DEST/.upload/lib/'*; do mv \"\$source\" '$DEST/lib/'; done"
fi
"${SCP_CMD[@]}" "${SSH_OPTIONS[@]}" "$model" "$BOARD:$DEST/.upload/supercombo.kmodel"
"${SSH_CMD[@]}" "${SSH_OPTIONS[@]}" "$BOARD" "mv '$DEST/.upload/supercombo.kmodel' '$DEST/model/supercombo.kmodel'"
"${SCP_CMD[@]}" "${SSH_OPTIONS[@]}" \
  params/calibration.json \
  params/yg_adaptive_cruise.json \
  params/yg_steering.json \
  params/yg_driving.json \
  params/recording.json \
  params/display.json \
  "$BOARD:$DEST/params.defaults/"
"${SSH_CMD[@]}" "${SSH_OPTIONS[@]}" "$BOARD" \
  "if test ! -e '$DEST/params/yg_adaptive_cruise.json' && test -e '$DEST/params/k7_yg_adaptive_cruise.json'; then cp '$DEST/params/k7_yg_adaptive_cruise.json' '$DEST/params/yg_adaptive_cruise.json'; fi; if test ! -e '$DEST/params/yg_steering.json' && test -e '$DEST/params/k7_yg_steering.json'; then cp '$DEST/params/k7_yg_steering.json' '$DEST/params/yg_steering.json'; fi; if test ! -e '$DEST/params/yg_driving.json' && test -e '$DEST/params/k7_yg_driving.json'; then cp '$DEST/params/k7_yg_driving.json' '$DEST/params/yg_driving.json'; fi"
"${SSH_CMD[@]}" "${SSH_OPTIONS[@]}" "$BOARD" \
  "for name in calibration.json yg_adaptive_cruise.json yg_steering.json yg_driving.json recording.json display.json; do test -e '$DEST/params/'\"\$name\" || cp '$DEST/params.defaults/'\"\$name\" '$DEST/params/'\"\$name\"; done"
"${SSH_CMD[@]}" "${SSH_OPTIONS[@]}" "$BOARD" "rm -rf '$DEST/.upload'; sync"
echo "Uploaded runtime files to $BOARD:$DEST"
