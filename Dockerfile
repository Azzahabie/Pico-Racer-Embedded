# Build environment for mtk3smp-rp2040.
# Produces .uf2 files identically regardless of host OS (Windows/WSL2, Intel Mac, Apple Silicon Mac).
# Flashing and serial monitoring still happen on the host OS, not in this container.

FROM ubuntu:24.04

ARG TARGETARCH
ARG ARM_TOOLCHAIN_VERSION=13.2.rel1

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    wget \
    xz-utils \
    ca-certificates \
    python3 \
    libncurses-dev \
    bear \
    && rm -rf /var/lib/apt/lists/*

# Pick the correct host build of the toolchain: x86_64 for Windows/WSL2 and Intel Mac,
# aarch64 for Apple Silicon Mac running native arm64 Linux containers.
RUN set -eux; \
    case "${TARGETARCH}" in \
        amd64) ARM_HOST=x86_64 ;; \
        arm64) ARM_HOST=aarch64 ;; \
        *) echo "Unsupported architecture: ${TARGETARCH}" && exit 1 ;; \
    esac; \
    wget -q "https://developer.arm.com/-/media/Files/downloads/gnu/${ARM_TOOLCHAIN_VERSION}/binrel/arm-gnu-toolchain-${ARM_TOOLCHAIN_VERSION}-${ARM_HOST}-arm-none-eabi.tar.xz" -O /tmp/arm-toolchain.tar.xz; \
    mkdir -p /opt/arm-toolchain; \
    tar -xf /tmp/arm-toolchain.tar.xz -C /opt/arm-toolchain --strip-components=1; \
    rm /tmp/arm-toolchain.tar.xz

ENV PATH="/opt/arm-toolchain/bin:${PATH}"

# Pico SDK, needed only for the CONSOLE=usb_cdc and WIFI=cyw43 build profiles.
# Pinned to 2.2.0 - the version GETTING_STARTED.md qualifies the port against.
# Only tinyusb, cyw43-driver and lwip are needed; mbedtls and btstack are skipped
# to save a large download, per GETTING_STARTED.md step 1b.
ARG PICO_SDK_TAG=2.2.0
RUN git clone -b ${PICO_SDK_TAG} https://github.com/raspberrypi/pico-sdk.git /opt/pico-sdk && \
    cd /opt/pico-sdk && \
    git submodule update --init lib/tinyusb && \
    git submodule update --init lib/cyw43-driver && \
    git submodule update --init lib/lwip

ENV PICO_SDK_PATH=/opt/pico-sdk

WORKDIR /workspace

# To get IntelliSense working in VS Code, run builds through bear instead of make
# directly, e.g. from inside build_make/: bear -- make -j8
# This writes compile_commands.json at the repo root, which the C/C++ extension
# picks up automatically.

CMD ["/bin/bash"]