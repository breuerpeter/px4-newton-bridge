FROM nvidia/cuda:13.0.1-runtime-ubuntu24.04

LABEL org.opencontainers.image.source="https://github.com/breuerpeter/px4-newton-bridge" \
      org.opencontainers.image.description="\
Base image for running PX4 SITL with the Newton simulator. \
Only includes dependencies strictly required for building POSIX targets. \
No simulator-specific dependencies."

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ca-certificates \
        cmake \
        g++ \
        git \
        make \
        ninja-build \
        python3-venv \
    && rm -rf /var/lib/apt/lists/*

COPY --from=ghcr.io/astral-sh/uv:latest /uv /usr/local/bin/uv

RUN uv pip install --system --break-system-packages --no-cache \
    "empy>=3.3,<4" \
    "jinja2>=2.8" \
    jsonschema \
    kconfiglib \
    pyros-genmsg \
    pyyaml

# Writable cache dirs (for any --user UID)
RUN mkdir -p /var/cache/warp && chmod 777 /var/cache/warp
RUN mkdir -p /var/cache/venv && chmod 777 /var/cache/venv
