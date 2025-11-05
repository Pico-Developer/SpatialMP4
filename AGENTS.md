# Repository Guidelines

## Project Structure & Module Organization
Core C++ sources live in `src/spatialmp4`, with corresponding headers under `spatialmp4/`. Pybind11 bridge code is under `bindings/`, while the published Python package pieces sit in `python/`. Reference assets are stored in `video/`, reusable build logic in `cmake/`, and helper scripts in `scripts/`. End-user docs and design notes are collected in `docs/`, and runnable walkthroughs reside in `examples/`.

## Build, Test, and Development Commands
Build FFmpeg and third-party dependencies once via `bash scripts/build_ffmpeg.sh` (or `pixi run build-ffmpeg`). Configure and compile the C++ library with `pixi run build`, which wraps the Ninja-based CMake invocation. For a clean rebuild, use `pixi run rebuild`. Run the C++ regression suite with `pixi run test` (expects `video/test.mp4` relative to `build/`), and execute Python coverage with `pixi run test-python`. When debugging locally outside Pixi, replicate the commands shown inside `pixi.toml`.

## Coding Style & Naming Conventions
We enforce clang-format 17 using the repository `.clang-format` (Google base style, two-space indentation, 120 character limit). Ensure each new C++ source file includes the license header inserted by the `insert-license` pre-commit hook. Function and type names should be in PascalCase, while variables use lower_snake_case; mirror existing reader/writer APIs. Python modules follow PEP 8, with snake_case functions and CapWords classes. Install the hooks with `pre-commit install` and run `pre-commit run --all-files` before uploading.

## Testing Guidelines
Prefer authoring new C++ tests next to the feature under `src/spatialmp4/tests` or augmenting `test_reader`. Tests should gate on fixtures in `video/` or clearly documented synthetic inputs. Python tests belong in `python/tests/` and must start with `test_`. Document any new assets and keep them under 5 MB where possible. Capture expected outputs or assertions that cover RGB and depth code paths when adding spatial data features.

## Commit & Pull Request Guidelines
Commit messages are short, present-tense imperatives (e.g., `add depth frame normalizer`). Group related changes per commit; avoid umbrella “update” descriptions. Pull requests must describe the motivation, list functional changes, and call out breaking API shifts. Reference tracked issues with `Fixes #ID` when applicable, include reproduction or validation commands (copy the `pixi run …` steps), and attach screenshots for visualization updates.
