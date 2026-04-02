# Docker

Run PX4 SITL with the Newton simulator in Docker. No native build dependencies required — only Docker.

## Prerequisites

- [Docker](https://docs.docker.com/get-docker/)

## Usage

```bash
./run_sitl.sh astro_max
```

The bridge dependencies are installed automatically into a cached virtual environment on the first run.
