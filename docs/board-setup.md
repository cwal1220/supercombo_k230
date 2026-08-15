# Board setup

[← Documentation index](../README.md)

For a freshly flashed board, install the build/runtime helper packages first:

```sh
apt-get update
apt-get install -y \
  ca-certificates \
  cmake \
  curl \
  g++ \
  git \
  libdrm-dev \
  libusb-1.0-0-dev \
  libopencv-dev \
  make \
  python3 \
  python3-pip
```

## Package purpose

- `g++`, `make`, `cmake`: board-native C/C++ build
- `libdrm-dev`: DRM headers used by the overlay/display path
- `libusb-1.0-0-dev`: optional panda USB/CAN bridge build
- `libopencv-dev`: OpenCV headers/libraries used by the overlay renderer
- `curl`, `ca-certificates`: `scripts/fetch_nncase_runtime.sh` download support
- `git`: fresh clone from GitHub
- `python3`: `k230_manager.py` and the K7 parameter web server
- `python3-pip`: FastAPI/uvicorn install

Install the parameter server dependencies:

```sh
python3 -m pip install -r scripts/requirements-param-server.txt
```

If the directory is copied to the board with all files already present, `git` is
not needed for building. It is only needed for a fresh repository checkout.

## Required image contents

The flashed image must already include the K230 camera/display devices and these
target runtime libraries:

- `libdisplay.so`
- `libv4l2-drm.so`
- `libdrm.so.2`

## Physical mounting

A printable windshield-mount bridge for the K230 and its LCD is documented in
[hardware/windshield_mount](hardware/windshield_mount/README.md).
