#!/usr/bin/env bash
set -euo pipefail

BOARD="${1:-root@192.168.219.115}"
DEST="${K230_BOARD_DIR:-/root/supercombo_native}"
RSH="${K230_RSYNC_RSH:-ssh -o PubkeyAuthentication=no -o PreferredAuthentications=password -o StrictHostKeyChecking=no}"

files=(
  supercombo.elf
  k230_camerad
  k230_modeld
  k230_overlay
  k230_manager.py
  k230_controlsd.py
)

if [ -x k230_pandad ]; then
  files+=(k230_pandad)
fi

rsync -av -e "$RSH" "${files[@]}" "$BOARD:$DEST/"
echo "Uploaded runtime files to $BOARD:$DEST"
