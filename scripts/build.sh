#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"
exec "$repo_root/scripts/with_limits.sh" colcon build --packages-select fast_lio \
  --executor sequential --cmake-args \
  "-DGTSAM_DIR=$repo_root/.deps/gtsam/lib/cmake/GTSAM" \
  -DCMAKE_BUILD_TYPE=Release '-DCMAKE_CXX_FLAGS_RELEASE=-O1 -DNDEBUG' "$@"
