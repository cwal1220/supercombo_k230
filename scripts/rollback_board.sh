#!/usr/bin/env bash
set -euo pipefail

BOARD="${1:-root@192.168.219.111}"
DEST="${K230_BOARD_DIR:-/root/supercombo_k230}"
RESTART_AFTER_ROLLBACK="${K230_RESTART_AFTER_ROLLBACK:-0}"
read -r -a SSH_CMD <<< "${K230_SSH:-ssh}"
SSH_OPTIONS=(
  -o PubkeyAuthentication=no
  -o PreferredAuthentications=password
  -o StrictHostKeyChecking=no
)

"${SSH_CMD[@]}" "${SSH_OPTIONS[@]}" "$BOARD" \
  "set -eu; test -d '$DEST/rollback' || { echo 'No rollback snapshot' >&2; exit 1; }; /etc/init.d/S35supercombo_k230 stop >/dev/null 2>&1 || true; for source in '$DEST/rollback/'*; do test -f \"\$source\" || continue; cp -p \"\$source\" '$DEST/'; done; test ! -f '$DEST/rollback/model/supercombo.kmodel' || cp -p '$DEST/rollback/model/supercombo.kmodel' '$DEST/model/supercombo.kmodel'; sync"

if [ "${RESTART_AFTER_ROLLBACK}" = "1" ]; then
  "${SSH_CMD[@]}" "${SSH_OPTIONS[@]}" "$BOARD" "/etc/init.d/S35supercombo_k230 start"
  echo "Rollback restored and started at $BOARD:$DEST"
else
  echo "Rollback restored at $BOARD:$DEST; service intentionally left stopped"
fi
