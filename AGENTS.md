# Repository Guidelines

## Project Structure & Module Organization
Core C++ sources live in `src/spatialmp4`, with helpers under `src/spatialmp4/utilities` and Google Test fixtures in `src/spatialmp4/reader_test.cc`. Python bindings are assembled in `bindings/spatialmp4.cpp`, while Python packaging and tests sit in `python/`. Reference assets (for example `video/test.mp4`) and temporary visualization outputs (`tmp_vis_depth/`) are kept at the repository root. Example end-to-end scripts are available under `examples/python` and double as smoke tests for new features.

## Build, Test, and Development Commands
Install third-party dependencies and FFmpeg once via `bash scripts/install_deps.sh` and `bash scripts/build_ffmpeg.sh`. A standard native build flow is `cmake -B build -S . -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON=OFF` followed by `cmake --build build`. When using Pixi, `pixi run build-ffmpeg`, `pixi run build`, and `pixi run rebuild` mirror those steps. Package the Python wheel with `pip install .` or `pixi run build` after enabling `-DBUILD_PYTHON=ON`.

## Coding Style & Naming Conventions
`.clang-format` enforces Google style with 2-space indentation, 120-character lines, and brace-on-same-line formatting. Run `pre-commit run --all-files` (configured for `clang-format` and license insertion) before submitting patches. C++ classes use PascalCase (e.g., `RandomAccessVideoReader`), member functions stay in UpperCamelCase, and local variables use snake_case. Python modules follow PEP 8 naming; prefer snake_case for functions and lower_case_with_underscores for files.

## Testing Guidelines
Enable the C++ test suite with `cmake -B build -S . -DBUILD_TESTING=ON && cmake --build build`, then execute `VIDEO=../video/test.mp4 ./build/test_reader`. The Pixi shortcut `pixi run test` wraps the same flow. Python tests live in `python/tests` and use pytest; run them with `pixi run test-python` or `pytest python/tests`. New tests should follow the `test_*.py` or `*_test.cc` naming pattern and cover both RGB-only and depth-enabled paths; set the `VIDEO` environment variable when a custom fixture is required.

## Commit & Pull Request Guidelines
History favors short, imperative commit subjects (for example, `set loglevel in reader`). Keep message bodies concise, reference related issues, and group mechanical formatting changes separately from functional updates. Pull requests should describe the feature or fix, outline test coverage (`pixi run test`, `pixi run test-python`, manual example scripts), and include screenshots or logs when touching visualization or CLI output. Ensure CI prerequisites (FFmpeg build, dependencies) are documented in the PR notes for new contributors.
