# tessera

`tessera` is a `no_std` Rust library for:
- spatial grids (`Grid`, `Grid3`, `HexGrid`)
- deterministic pathfinding (`A*`, `JPS` API, Dijkstra, flow fields)
- visibility and connectivity (`LOS`, FOV, flood fill)
- procedural generation (BSP, cellular automata, drunkard walk, Poisson)
- bridge utilities for integrating tile maps and collision AABBs

The implementation follows `IMPLEMENTATION.md`.

## Live Showcase

- Interactive WASM demo: <https://robdavenport.github.io/tessera/>
- Demo source: `demo-wasm/`
- GitHub Pages workflow: `.github/workflows/pages.yml`

If this is your first deployment, set `Settings -> Pages -> Build and deployment -> Source` to `GitHub Actions`.
After pushing to `main`, GitHub Actions builds the WASM demo and deploys it to Pages automatically.

## Project Layout

- `src/`: core library modules
- `demo-wasm/`: browser demo (WASM + Canvas) with four tabs:
  - Pathfinding
  - Field of View
  - Dungeon Generator
  - Hex Grid
- `.github/workflows/pages.yml`: GitHub Pages deployment for the WASM demo
- `vendor/`: local path dependencies for `rand_core` and `libm`
- `vendor-crates/`: vendored crates.io sources (including `wasm-bindgen` family)
- `.cargo/config.toml`: forces Cargo to use `vendor-crates/` instead of crates.io

## Build & Verify

From repo root:

```bash
cargo test --target x86_64-pc-windows-msvc
cargo build --target wasm32-unknown-unknown --release
cargo clippy --target x86_64-pc-windows-msvc -- -D warnings
```

## Run the WASM Showcase Locally

Prerequisites:
- `wasm-pack`
- a static file server

```bash
cd demo-wasm
wasm-pack build --target web --release --out-dir pkg
```

Then serve `demo-wasm/www/` with any HTTP server (for example `python -m http.server` from `demo-wasm/www`) and open it in your browser.

## Offline Build Notes

This repository is configured for offline Cargo resolution:
- crates.io is replaced with local `vendor-crates/`
- `rand_core` and `libm` are path dependencies under `vendor/`

As long as `vendor/`, `vendor-crates/`, and `.cargo/config.toml` are present, `cargo` and `wasm-pack` builds do not need network access for Rust dependencies.

## GitHub Pages

The workflow in `.github/workflows/pages.yml` builds `demo-wasm`, publishes the `www/` app + generated `pkg/`, and deploys to GitHub Pages on pushes to `main`.
