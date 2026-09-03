#!/usr/bin/env bash
# nncase's x86 .NET runtime needs these flags under Rosetta on Apple Silicon.
set -euo pipefail
repo="$(cd "$(dirname "$0")/../.." && pwd)"
exec docker run --rm --platform linux/amd64 --ulimit core=0 \
  -e DOTNET_EnableWriteXorExecute=0 -e COMPlus_EnableWriteXorExecute=0 \
  -e COMPlus_TieredCompilation=0 -e COMPlus_ReadyToRun=0 -e COMPlus_ZapDisable=1 \
  -e OMP_NUM_THREADS=4 -v "${repo}:/work" -w /work \
  "${DOCKER_IMAGE:-supercombo-nncase-k230:2.11.0-sdk}" "$@"
