# PX4-Newton Bridge

- `newton/` is a git submodule (the Newton physics engine). `px4_newton_bridge/__init__.py` inserts it into `sys.path` — do not move these imports above the path manipulation.
- Bridge and PX4 run in lockstep: send sensor data, block waiting for actuator controls (MAVLink over TCP port 4560). A 2-second timeout detects PX4 disconnection.
- Vehicle models are YAML configs in `px4_newton_bridge/vehicles/` pointing to URDF files.
- gRPC API (`newton_api/`) is optional, enabled via `config.yaml`. After editing `proto/`, regenerate with `uv run --no-project --with grpcio-tools python px4_newton_bridge/_generate_proto.py`. CI verifies generated code is up to date.
- SITL can be run locally (`make px4_sitl_newton newton_astro_max` from PX4 root) or with Docker (`./run_docker.sh astro_max`). Both install bridge dependencies automatically via `uv`.
- Tests require a GPU. They run on RunPod via the `gpu-sitl-test.yml` workflow.

## Commits

Conventional commits with required scope: `type(scope): description`.

release-please uses commit types to determine version bumps:
- `feat` → minor (0.1.0 → 0.2.0)
- `fix` → patch (0.1.0 → 0.1.1)
- `feat!` or `BREAKING CHANGE` footer → major
- `chore`, `docs`, `ci`, `refactor`, `test` → included in changelog but no version bump

Run `uvx pre-commit run -a` to lint/format before committing. Use `uv` for all Python commands.

## PRs

- Create a feature branch — never commit directly to `main`.
- Use the PR template in `.github/PULL_REQUEST_TEMPLATE.md`.
- pre-commit.ci runs automatically on PRs. CI also checks the lockfile and docs build.
