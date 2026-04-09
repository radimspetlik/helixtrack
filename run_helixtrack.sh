#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_PATH="${1:-${SCRIPT_DIR}/config_tracker_smoke.yaml}"

if [ $# -gt 0 ]; then
  shift
fi

if [ ! -x "${SCRIPT_DIR}/build/helixtrack" ]; then
  "${SCRIPT_DIR}/compile_build.sh" release
fi

mkdir -p "${SCRIPT_DIR}/output"

exec "${SCRIPT_DIR}/rci_job_singularity.sh" \
  "${SCRIPT_DIR}/build/helixtrack" \
  "${CONFIG_PATH}" \
  "$@"
