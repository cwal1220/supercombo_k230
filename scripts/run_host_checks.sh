#!/usr/bin/env bash
# 호스트에서 도는 자체 검사를 전부 빌드하고 실행한다. 보드도 K230 툴체인도
# 필요 없다. check_adaptive_cruise가 params/를 상대경로로 읽으므로 저장소
# 루트에서 실행한다.
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

build_dir="${K230_HOST_BUILD_DIR:-build-host}"
jobs="${JOBS:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 2)}"

checks=(
  check_adaptive_cruise
  check_control_replay
  check_departure_alert
  check_k230_can_queue
  check_lateral_mpc
  check_model_output_parser
  check_panda_can_codec
  verify_calibration_equivalence
)

cmake -S . -B "${build_dir}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DSUPERCOMBO_BUILD_RUNTIME=OFF \
  -DSUPERCOMBO_BUILD_DIAGNOSTICS=ON >/dev/null
cmake --build "${build_dir}" -j"${jobs}" "${checks[@]/#/--target=}" >/dev/null

failed=0
for check in "${checks[@]}"; do
  printf '%-32s ' "${check}"
  if output="$("${build_dir}/bin/${check}" 2>&1)"; then
    echo "OK"
  else
    echo "FAIL"
    printf '%s\n' "${output}" | sed 's/^/    /'
    failed=$((failed + 1))
  fi
done

if [ "${failed}" -ne 0 ]; then
  echo "${failed} check(s) failed" >&2
  exit 1
fi
echo "all host checks passed"
