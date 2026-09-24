# 스크립트

호스트에서 빌드·배포·검사에 쓰는 스크립트와, 보드에 설치돼 런타임과 함께 도는 Python이 있다.
보드용 파일은 `upload_to_board.sh`(또는 `cmake --install`)가 보드 설치 디렉터리의 최상위에
실행 파일과 나란히 둔다.

## 호스트

| 스크립트 | 사용 | 하는 일 |
| --- | --- | --- |
| `configure_k230_macos.sh` | `[빌드 디렉터리]` | macOS에서 K230 교차 빌드를 구성한다(기본 `build/`). 이후 `cd build && make -j2` |
| `fetch_nncase_runtime.sh` | | nncase K230 런타임과 gsl-lite를 SHA256을 확인해 받아 `deps/`를 만든다. 보드에서도 돈다 |
| `upload_to_board.sh` | `[root@보드]` | 빌드한 런타임, 보드용 Python, 모델, UI 스프라이트, 파라미터 기본값을 보드에 올린다 |
| `run_host_tests.sh` | | 호스트 단위 테스트를 빌드하고 `ctest`로 전부 돌린다([gtest/](../gtest/README.md)) |
| `build_supercombo_model.sh` | | openpilot v0.9.4 ONNX를 K230 kmodel로 빌드한다(nncase Docker 이미지 필요, [models/](../models/README.md)) |

환경 변수로 바꿀 수 있는 것:

- `configure_k230_macos.sh`: `K230_WORKSPACE_DIR`, `K230_XUANTIE_TOOLCHAIN_DIR`, `K230_RISCV_LD`,
  `K230_BOARD_LIBS_DIR`, `SUPERCOMBO_BUILD_PANDA`, `SUPERCOMBO_BUILD_DIAGNOSTICS` 등.
  경로는 구성 전에 모두 있는지 확인한다.
- `upload_to_board.sh`: `K230_SSH`/`K230_SCP`(예: `sshpass -p ... ssh`), `K230_BUILD_DIR`
  (기본 `build`), `K230_BIN_DIR`, `K230_BOARD_DIR`(기본 `/root/supercombo_k230`). 보드의
  `params/`는 덮어쓰지 않고, 기본값은 `params.defaults/`에 두어 없는 파일만 채운다. 보드
  이미지에 `/etc/init.d/S35supercombo_k230`이 없으면 올리지 않는다.
- `run_host_tests.sh`: `K230_HOST_BUILD_DIR`(기본 `build-host`), `JOBS`.
- `build_supercombo_model.sh`: `DOCKER_IMAGE`, `OPENPILOT_TAG`, `SOURCE_ONNX`, `PYTHON_BIN`.

자세한 빌드와 배포 절차는 [Build and deploy](../docs/build-and-deploy.md)에 있다.

## 보드

| 파일 | 사용 | 하는 일 |
| --- | --- | --- |
| `k230_manager.py` | `./k230_manager.py [supercombo.kmodel] [debug_mode]` | 런타임 감시자. 프로세스를 순서대로 띄우고 죽으면 다시 띄운다. 이미지의 `S35supercombo_k230`이 부팅 때 실행한다 |
| `k230_param_server.py` | `[--host 주소] [--port 포트]` | 파라미터 편집 웹 서버(FastAPI, 기본 `0.0.0.0:8080`). 매니저가 함께 띄운다 |
| `display_control.py` | (모듈) | LCD 백라이트 제어. 파라미터 서버가 `display.json`을 적용할 때 쓴다 |
| `requirements-param-server.txt` | `python3 -m pip install -r ...` | 파라미터 서버 의존성(fastapi, uvicorn) |

어떤 프로세스를 띄울지와 환경 변수는 [분할 런타임](../docs/runtime.md)과
[런타임 옵션](../docs/runtime-options.md)에 있다.

## 작성 규칙

- 첫머리 주석(Python은 모듈 docstring)에 용도를 한글로 적고 `사용:` 줄로 끝낸다.
- 셸 스크립트는 `set -euo pipefail`로 시작하고, 저장소 루트를 스스로 찾아 어디서 불러도
  돌게 한다. 보드에서도 도는 스크립트는 POSIX `sh`로 쓴다.
