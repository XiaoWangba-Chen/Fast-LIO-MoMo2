#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${FASTLIO_LIMITED:-0}" != 1 ]]; then
  exec "$repo_root/scripts/with_limits.sh" env FASTLIO_LIMITED=1 "$0" "$@"
fi
dep_root="${repo_root}/.deps"
mkdir -p "$dep_root"
archive="$dep_root/gtsam-4.2.tar.gz"
if [[ ! -f "$archive" ]]; then
  curl --fail --location --retry 2 https://codeload.github.com/borglab/gtsam/tar.gz/refs/tags/4.2 --output "$archive"
fi
printf '%s  %s\n' '9ff8846d0a83a245c284cb5760ec2d74535ef9b5885183ccfefd7ff122eba60e' "$archive" | sha256sum --check
if [[ ! -d "$dep_root/gtsam-4.2" ]]; then tar -xzf "$archive" -C "$dep_root"; fi
cmake -S "$dep_root/gtsam-4.2" -B "$dep_root/gtsam-build" \
  -DCMAKE_INSTALL_PREFIX="$dep_root/gtsam" -DCMAKE_BUILD_TYPE=Release \
  '-DCMAKE_INSTALL_RPATH=$ORIGIN' \
  -DCMAKE_CXX_FLAGS_RELEASE='-O1 -DNDEBUG' \
  -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_UNSTABLE=OFF -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_WITH_TBB=OFF -DGTSAM_BUILD_PYTHON=OFF
cmake --build "$dep_root/gtsam-build" --parallel "${FASTLIO_BUILD_JOBS:-1}"
cmake --install "$dep_root/gtsam-build"
git -C "$repo_root" submodule update --init
printf 'GTSAM ready. Build with resource limits: %s/scripts/build.sh\n' "$repo_root"
