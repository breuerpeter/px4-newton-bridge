# Development

## Pre-commit hooks

This project uses [pre-commit](https://pre-commit.com/) to run code quality checks before each commit. The hooks enforce:

- **ruff** — Python linting and formatting
- **uv-lock** — keeps `uv.lock` in sync with `pyproject.toml`
- **typos** — catches common misspellings
- **conventional-pre-commit** — enforces [conventional commit](https://www.conventionalcommits.org/) messages

### Setup

Install the hooks (one-time):

```bash
uvx pre-commit install
uvx pre-commit install --hook-type commit-msg
```

### Usage

Hooks run automatically on `git commit`. To run them manually against all files:

```bash
uvx pre-commit run --all-files
```

If a hook modifies a file (e.g. ruff auto-formats), the commit is aborted. Stage the changes and commit again.
