# supercombo kmodel 컴파일용 nncase 2.11.0 이미지 (linux/amd64, Rosetta로 실행)
FROM --platform=linux/amd64 mcr.microsoft.com/dotnet/runtime:7.0-bullseye-slim AS dotnet
FROM --platform=linux/amd64 python:3.10-slim-bullseye

COPY --from=dotnet /usr/share/dotnet /usr/share/dotnet
ENV DOTNET_ROOT=/usr/share/dotnet
ENV PATH="${DOTNET_ROOT}:${PATH}"

RUN apt-get update \
    && apt-get install -y --no-install-recommends libgomp1 libicu67 \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir \
    nncase==2.11.0 \
    nncase-kpu==2.11.0 \
    onnx \
    numpy

ENV NNCASE_PLUGIN_PATH=/usr/local/lib/python3.10/site-packages/
