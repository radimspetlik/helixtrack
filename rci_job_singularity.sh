#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORK_DIR="${SCRIPT_DIR}/.container-work"
TMP_DIR="${SCRIPT_DIR}/.container-tmp"
cd "${SCRIPT_DIR}" || exit 1

export PYTHONPATH="${SCRIPT_DIR}:${PYTHONPATH:-}"

DATA_DIR="${HELIXTRACK_DATA_DIR:-}"
SCRATCH_DIR="${HELIXTRACK_SCRATCH_DIR:-}"
SINGULARITY_IMAGE="${HELIXTRACK_CONTAINER_IMAGE:-${SCRIPT_DIR}/helixtrack.sif}"
RUNTIME="${HELIXTRACK_CONTAINER_RUNTIME:-$(command -v apptainer || command -v singularity || true)}"

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

mkdir -p "${WORK_DIR}"
mkdir -p "${TMP_DIR}"

if [ $# -eq 0 ]; then
  PROGRAM_WITH_ARGS=("${SCRIPT_DIR}/build/helixtrack" "${SCRIPT_DIR}/config_tracker.yaml")
  echo "zero arguments, using default: ${PROGRAM_WITH_ARGS[*]}"
else
  PROGRAM_WITH_ARGS=("$@")
fi

if [[ "${PROGRAM_WITH_ARGS[0]}" == ./* ]]; then
  PROGRAM_WITH_ARGS[0]="${SCRIPT_DIR}/${PROGRAM_WITH_ARGS[0]#./}"
fi

if [ "${#PROGRAM_WITH_ARGS[@]}" -ge 2 ] && [[ "${PROGRAM_WITH_ARGS[1]}" != /* ]]; then
  PROGRAM_WITH_ARGS[1]="${SCRIPT_DIR}/${PROGRAM_WITH_ARGS[1]}"
fi

BIND_ARGS=(
  --bind "${SCRIPT_DIR}:${SCRIPT_DIR}"
)

if [ -n "${DATA_DIR}" ] && [ -d "${DATA_DIR}" ]; then
  BIND_ARGS+=(--bind "${DATA_DIR}:${DATA_DIR}")
fi

if [ -d "${SCRATCH_DIR}" ]; then
  BIND_ARGS+=(--bind "${SCRATCH_DIR}:${SCRATCH_DIR}")
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
  "${PROGRAM_WITH_ARGS[@]}"
