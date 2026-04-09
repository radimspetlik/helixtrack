#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT_IMAGE="${HELIXTRACK_CONTAINER_IMAGE:-${SCRIPT_DIR}/helixtrack.sif}"
RUNTIME="${HELIXTRACK_CONTAINER_RUNTIME:-$(command -v apptainer || command -v singularity || true)}"

if [ -z "${RUNTIME}" ]; then
  echo "Neither apptainer nor singularity is available."
  exit 1
fi

mkdir -p "$(dirname "${OUTPUT_IMAGE}")"

BUILD_ARGS=()
if [ "${HELIXTRACK_CONTAINER_FAKEROOT:-0}" = "1" ]; then
  BUILD_ARGS+=(--fakeroot)
fi

"${RUNTIME}" build "${BUILD_ARGS[@]}" "${OUTPUT_IMAGE}" "${SCRIPT_DIR}/singularity.def"
