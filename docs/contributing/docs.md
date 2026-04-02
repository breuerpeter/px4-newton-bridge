# Documentation

The docs are built with [MkDocs Material](https://squidfunk.github.io/mkdocs-material/) and deployed to GitHub Pages automatically on push to `main`.

## Prerequisites

- [uv](https://docs.astral.sh/uv/getting-started/installation/)

## Local preview

```bash
uvx --with mkdocs-material mkdocs serve
```

Then open [http://localhost:8000](http://localhost:8000). Changes to Markdown files are reflected live.

## Build

```bash
uvx --with mkdocs-material mkdocs build
```

Output is written to `site/`.
