#!/usr/bin/env bash
# Convenience launcher for examples/python/visualize_rerun_full.py
#
# Sets DYLD_FALLBACK_LIBRARY_PATH so the Homebrew OpenEXR/Imath/OpenCV libs
# load on macOS (their dylib id strings contain @@HOMEBREW_PREFIX@@ unless
# `brew postinstall openexr` has been run), and PYTHONPATH so the locally
# built spatialmp4 module is importable without `pip install .`.
#
# Usage:
#   ./run_visualize_rerun_full.sh path/to/spatial.mp4 [extra args...]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"

: "${SPATIALMP4_PYTHON_BIN:=/tmp/spm4_venv/bin/python}"
: "${SPATIALMP4_SO_DIR:=${REPO_DIR}/build/host_py/python}"

if [ "$#" -lt 1 ]; then
  echo "usage: $0 <video.mp4> [extra args]" >&2
  exit 1
fi

export DYLD_FALLBACK_LIBRARY_PATH="/opt/homebrew/lib${DYLD_FALLBACK_LIBRARY_PATH:+:${DYLD_FALLBACK_LIBRARY_PATH}}"
export PYTHONPATH="${SPATIALMP4_SO_DIR}${PYTHONPATH:+:${PYTHONPATH}}"

exec "${SPATIALMP4_PYTHON_BIN}" "${SCRIPT_DIR}/visualize_rerun_quest.py" "$@"
