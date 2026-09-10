#!/bin/sh
set -eu

VERSION="2.11.0"
ARCHIVE="nncase_k230_v${VERSION}_runtime_linux.tgz"
URL="https://github.com/kendryte/nncase/releases/download/v${VERSION}/${ARCHIVE}"
SHA256="28680932ac879d8591fbaaaab7b8c1ee2d305c2a82471fb2f38c449316cfb91f"

# nncase 공개 헤더 27개가 <gsl/gsl-lite.hpp>를 요구하는데 릴리스 타르볼에는
# 없다. 같은 스크립트로 받아 deps/를 자족적으로 만든다.
GSL_VERSION="0.37.0"
GSL_HEADER="gsl-lite-${GSL_VERSION}.hpp"
GSL_URL="https://raw.githubusercontent.com/gsl-lite/gsl-lite/v${GSL_VERSION}/include/gsl/gsl-lite.hpp"
GSL_SHA256="656894aefe55fd316a6b2e777fc7e103898e41ad501abeeb7c4a5c3af82ed7be"

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_DIR}"
mkdir -p deps .cache

verify() {
  file="$1"
  expected="$2"
  if command -v sha256sum >/dev/null 2>&1; then
    echo "${expected}  ${file}" | sha256sum -c -
  else
    actual="$(shasum -a 256 "${file}" | awk '{print $1}')"
    test "${actual}" = "${expected}"
  fi
}

if [ ! -f ".cache/${ARCHIVE}" ]; then
  curl -L --fail --retry 3 -o ".cache/${ARCHIVE}" "${URL}"
fi
verify ".cache/${ARCHIVE}" "${SHA256}"

if [ ! -f ".cache/${GSL_HEADER}" ]; then
  curl -L --fail --retry 3 -o ".cache/${GSL_HEADER}" "${GSL_URL}"
fi
verify ".cache/${GSL_HEADER}" "${GSL_SHA256}"

rm -rf deps
mkdir -p deps
tar -xzf ".cache/${ARCHIVE}" -C deps --strip-components=1
mkdir -p deps/include/gsl
cp ".cache/${GSL_HEADER}" deps/include/gsl/gsl-lite.hpp
