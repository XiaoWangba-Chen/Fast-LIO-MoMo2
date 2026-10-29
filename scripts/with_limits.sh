#!/usr/bin/env bash
# Keep builds from competing with the desktop. Limits cover all descendants.
set -euo pipefail
export CMAKE_BUILD_PARALLEL_LEVEL=1
export MAKEFLAGS=-j1
if [[ $# -eq 0 ]]; then printf 'Usage: %s command [args...]\n' "$0" >&2; exit 2; fi
if systemctl --user is-system-running >/dev/null 2>&1; then
  exec systemd-run --user --scope --quiet \
    -p "CPUQuota=${FASTLIO_CPU_QUOTA:-50%}" \
    -p "MemoryHigh=${FASTLIO_MEMORY_HIGH:-2800M}" \
    -p "MemoryMax=${FASTLIO_MEMORY_MAX:-3G}" \
    -p MemorySwapMax=0 -p IOWeight=10 \
    nice -n 15 ionice -c 3 "$@"
fi
# No user systemd: retain one build job, low priorities and one allowed CPU.
allowed_cpu="$(python3 -c 'import os; print(max(os.sched_getaffinity(0)))')"
exec nice -n 15 ionice -c 3 taskset -c "$allowed_cpu" "$@"
