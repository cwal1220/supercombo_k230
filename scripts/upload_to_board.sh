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
  -o PubkeyAuthentication=no
  -o PreferredAuthentications=password
  -o StrictHostKeyChecking=no
)

runtime_files=(
  "${BIN_DIR}/supercombo.elf"
  "${BIN_DIR}/k230_camerad"
  "${BIN_DIR}/k230_modeld"
  "${BIN_DIR}/k230_overlay"
  scripts/k230_manager.py
  scripts/k7_param_server.py
  scripts/requirements-param-server.txt
)
model="models/supercombo.kmodel"

if [ -x "${BIN_DIR}/k230_pandad" ]; then
  runtime_files+=("${BIN_DIR}/k230_pandad" "${BIN_DIR}/k230_k7_controlsd")
fi

for runtime_file in "${runtime_files[@]}"; do
  if [ ! -f "${runtime_file}" ]; then
    echo "Missing runtime file: ${runtime_file}" >&2
    exit 1
  fi
done

"${SSH_CMD[@]}" "${SSH_OPTIONS[@]}" "$BOARD" \
  "test -x /etc/init.d/S35supercombo_k230 || { echo 'Missing image-provided /etc/init.d/S35supercombo_k230' >&2; exit 1; }; rm -f /etc/init.d/S95supercombo_k230; mkdir -p '$DEST/model' '$DEST/params' '$DEST/params.defaults'"
"${SCP_CMD[@]}" "${SSH_OPTIONS[@]}" "${runtime_files[@]}" "$BOARD:$DEST/"
if [ -x "${BIN_DIR}/k230_k7_controlsd" ]; then
  "${SSH_CMD[@]}" "${SSH_OPTIONS[@]}" "$BOARD" "mkdir -p '$DEST/lib'"
  "${SCP_CMD[@]}" "${SSH_OPTIONS[@]}" \
    deps/acados/lateral_solver/libacados_ocp_solver_lat.so \
    deps/acados/riscv64/lib/libacados.so \
    deps/acados/riscv64/lib/libblasfeo.so \
    deps/acados/riscv64/lib/libhpipm.so \
    deps/acados/riscv64/lib/libqpOASES_e.so.3.1 \
    "$BOARD:$DEST/lib/"
fi
"${SCP_CMD[@]}" "${SSH_OPTIONS[@]}" "$model" "$BOARD:$DEST/model/"
"${SCP_CMD[@]}" "${SSH_OPTIONS[@]}" \
  params/calibration.json \
  params/k7_yg_adaptive_cruise.json \
  params/k7_yg_steering.json \
  params/k7_yg_driving.json \
  "$BOARD:$DEST/params.defaults/"
"${SSH_CMD[@]}" "${SSH_OPTIONS[@]}" "$BOARD" \
  "for name in calibration.json k7_yg_adaptive_cruise.json k7_yg_steering.json k7_yg_driving.json; do test -e '$DEST/params/'\"\$name\" || cp '$DEST/params.defaults/'\"\$name\" '$DEST/params/'\"\$name\"; done"
"${SSH_CMD[@]}" "${SSH_OPTIONS[@]}" "$BOARD" "sync"
echo "Uploaded runtime files to $BOARD:$DEST"
