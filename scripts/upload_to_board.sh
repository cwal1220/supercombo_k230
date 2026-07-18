#!/usr/bin/env bash
set -euo pipefail

BOARD="${1:-root@192.168.219.111}"
DEST="${K230_BOARD_DIR:-/root/supercombo_k230}"
read -r -a SSH_CMD <<< "${K230_SSH:-ssh}"
read -r -a SCP_CMD <<< "${K230_SCP:-scp}"
SSH_OPTIONS=(
  -o PubkeyAuthentication=no
  -o PreferredAuthentications=password
  -o StrictHostKeyChecking=no
)

runtime_files=(
  supercombo.elf
  k230_camerad
  k230_modeld
  k230_overlay
  k230_manager.py
  k230_controlsd.py
)
model="final_k230_model/model/supercombo.kmodel"

if [ -x k230_pandad ]; then
  runtime_files+=(k230_pandad)
fi

"${SSH_CMD[@]}" "${SSH_OPTIONS[@]}" "$BOARD" "mkdir -p '$DEST/model'"
"${SCP_CMD[@]}" "${SSH_OPTIONS[@]}" "${runtime_files[@]}" "$BOARD:$DEST/"
"${SCP_CMD[@]}" "${SSH_OPTIONS[@]}" "$model" "$BOARD:$DEST/model/"
echo "Uploaded runtime files to $BOARD:$DEST"
