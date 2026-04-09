#!/usr/bin/env bash

set -euo pipefail

if [ "$#" -ne 1 ] || ! [[ "$1" == "release" || "$1" == "debug" ]]; then
  echo "Usage: $0 release|debug"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
WORK_DIR="${SCRIPT_DIR}/.container-work"
TMP_DIR="${SCRIPT_DIR}/.container-tmp"
DATA_DIR="${HELIXTRACK_DATA_DIR:-}"
SINGULARITY_IMAGE="${HELIXTRACK_CONTAINER_IMAGE:-${SCRIPT_DIR}/helixtrack.sif}"
RUNTIME="${HELIXTRACK_CONTAINER_RUNTIME:-$(command -v apptainer || command -v singularity || true)}"
BUILD_JOBS="${HELIXTRACK_BUILD_JOBS:-24}"

if [ -z "${RUNTIME}" ]; then
  echo "Neither apptainer nor singularity is available."
  exit 1
fi

if [ ! -f "${SINGULARITY_IMAGE}" ]; then
  echo "Container image not found: ${SINGULARITY_IMAGE}"
  echo "Set HELIXTRACK_CONTAINER_IMAGE to a valid Apptainer or Singularity image."
  exit 1
fi

unset TMUX
unset TMUX_PANE

mkdir -p "${BUILD_DIR}"
mkdir -p "${WORK_DIR}"
mkdir -p "${TMP_DIR}"

declare -a BIND_ARGS
BIND_ARGS=(--bind "${SCRIPT_DIR}:${SCRIPT_DIR}")
if [ -n "${DATA_DIR}" ] && [ -d "${DATA_DIR}" ]; then
  BIND_ARGS+=(--bind "${DATA_DIR}:${DATA_DIR}")
fi
if [ -f /etc/localtime ]; then
  BIND_ARGS+=(--bind /etc/localtime:/etc/localtime)
fi

exec "${RUNTIME}" exec \
  --cleanenv \
  --containall \
  --no-home \
  --no-mount tmp \
  --env "TMPDIR=${TMP_DIR}" \
  --env "TEMP=${TMP_DIR}" \
  --env "TMP=${TMP_DIR}" \
  --workdir "${WORK_DIR}" \
  --pwd "${SCRIPT_DIR}" \
  "${BIND_ARGS[@]}" \
  "${SINGULARITY_IMAGE}" \
  bash -lc "cmake -Wno-dev -S '${SCRIPT_DIR}' -B '${BUILD_DIR}' -DCMAKE_BUILD_TYPE=${1^} && cmake --build '${BUILD_DIR}' --parallel ${BUILD_JOBS} --clean-first"
