#!/bin/sh
set -eu

VERSION="2.11.0"
ARCHIVE="nncase_k230_v${VERSION}_runtime_linux.tgz"
URL="https://github.com/kendryte/nncase/releases/download/v${VERSION}/${ARCHIVE}"
SHA256="28680932ac879d8591fbaaaab7b8c1ee2d305c2a82471fb2f38c449316cfb91f"

cd "$(dirname "$0")"
mkdir -p deps .cache

if [ ! -f ".cache/${ARCHIVE}" ]; then
  curl -L --fail --retry 3 -o ".cache/${ARCHIVE}" "${URL}"
fi

if command -v sha256sum >/dev/null 2>&1; then
  echo "${SHA256}  .cache/${ARCHIVE}" | sha256sum -c -
else
  actual="$(shasum -a 256 ".cache/${ARCHIVE}" | awk '{print $1}')"
  test "${actual}" = "${SHA256}"
fi

rm -rf deps
mkdir -p deps
tar -xzf ".cache/${ARCHIVE}" -C deps --strip-components=1
