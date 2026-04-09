# HelixTrack

Official implementation of:

**HelixTrack: Event-Based Tracking and RPM Estimation of Propeller-like Objects**  
Radim Spetlik, Michal Pliska, Vojtech Vrba, Jiri Matas  
CVPR Findings 2026

## Overview

HelixTrack is an event-driven tracker for propeller-like objects. It back-warps each event from the image plane into a rotor plane, tracks phase with a per-event filter, and refines pose with batched Gauss-Newton updates. The method outputs both geometry and instantaneous rotational speed from raw event streams.

The implementation follows the same method names used in the paper:

- image-to-rotor-plane homography
- per-event phase tracking
- wrapped helical phase residual
- batched pose refinement
- polarity alignment
- soft annulus barriers
- Balanced Tip Occupancy

## Repository Contents

- `main.cpp`: runtime loop, tracker wiring, metrics collection, and visualization
- `helixtrack_pose.h`: homography parameterization and analytic back-warp Jacobians
- `helixtrack_tracker.h`: phase tracker and batched pose refinement
- `metrics_writer.cpp`: HDF5 export of run outputs and summary metrics
- `recording_setup.cpp`: recording-specific initialization and RPM alignment loading
- `compile_build.sh`: build wrapper that compiles the project inside the container image
- `run_helixtrack.sh`: runtime wrapper for containerized execution
- `singularity.def`: OpenEB-based Apptainer or Singularity definition file
- `singularity_build.sh`: helper script that builds the container image
- `config_tracker.yaml`: full example configuration
- `config_tracker_smoke.yaml`: short smoke-test configuration

This repository is intentionally limited to the tracker and evaluation code. Private data loggers, dataset creation tools, and recording infrastructure are not included.

## Method Summary

HelixTrack maintains two coupled states:

- a homography that maps image coordinates into rotor-plane coordinates
- a phase state made of phase, angular speed, and angular acceleration

For each incoming event:

1. The event is back-warped into rotor-plane coordinates.
2. A wrapped helical phase residual is computed from rotor azimuth.
3. The phase state is updated with a per-event filter.
4. The event contributes phase, radial, polarity, and band terms to a batched Gauss-Newton system.

When enough support has accumulated, the batched solve updates the homography. RPM is obtained directly from the estimated angular speed.

## Dependencies

The code depends on:

- CMake 3.20+
- a C++17 compiler
- Eigen3
- OpenCV
- HDF5
- yaml-cpp
- OpenEB 5.2.0 or a compatible install that provides the `MetavisionSDK`
  CMake package and the `base`, `core`, `stream`, and `ui` components used by
  the code

We cannot redistribute the event-camera SDK sources or binaries in this repository.
The recommended setup path is the open-source OpenEB SDK:

https://github.com/prophesee-ai/openeb

OpenEB installs the `MetavisionSDK` and `MetavisionHAL` CMake packages and the
corresponding C++ headers used by this code.

## Quick Start

### Native build

If your machine already has the required SDK and libraries installed:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
./build/helixtrack config_tracker_smoke.yaml
```

### Apptainer or Singularity workflow

The recommended setup path is the OpenEB-based container defined in
`singularity.def`. The image build installs the system packages, pybind11, and
OpenEB needed by HelixTrack.

Build the image:

```bash
bash singularity_build.sh
```

By default this writes `helixtrack.sif` into the repository root.

Useful image-build environment variables:

- `HELIXTRACK_CONTAINER_IMAGE`: output image path
- `HELIXTRACK_CONTAINER_RUNTIME`: explicit runtime binary, for example `apptainer`
- `HELIXTRACK_CONTAINER_FAKEROOT=1`: build with `--fakeroot` when your system requires it
- `HELIXTRACK_DATA_DIR`: optional data directory to bind into the container at runtime

Set the runtime image explicitly when compiling or running:

```bash
export HELIXTRACK_CONTAINER_IMAGE=/path/to/helixtrack.sif
```

Useful runtime environment variables:

- `HELIXTRACK_CONTAINER_RUNTIME`: explicit runtime binary, for example `apptainer`
- `HELIXTRACK_DATA_DIR`: data root to bind into the container
- `HELIXTRACK_SCRATCH_DIR`: optional scratch directory to bind for runtime I/O
- `HELIXTRACK_BUILD_JOBS`: parallel build jobs used by `compile_build.sh`

Build and run:

```bash
./compile_build.sh release
./run_helixtrack.sh config_tracker_smoke.yaml
```

The default run script writes outputs under `output/`.

The container workflow does not vendor the event-camera SDK sources in this
repository. Instead, `singularity.def` fetches and installs OpenEB during image
build. If you prefer a native setup, install OpenEB yourself and then use the
native CMake build shown above.

## Configuration

The YAML configuration exposes the main method controls directly:

- initial pose and initial RPM
- blade count
- phase concentration `tracker_kappa`
- white-jerk process noise `tracker_q_jerk`
- informative ring bounds `tracker_r_inner` and `tracker_r_outer`
- phase and radial term weights
- polarity alignment parameters
- soft annulus barrier parameters
- Balanced Tip Occupancy parameters
- per-parameter Gauss-Newton step sizes

The shipped YAML files are templates. Replace `recording_filepath` with your own
recording before running. Use `config_tracker_smoke.yaml` for a short validation
run and `config_tracker.yaml` for the full setup.

## Data

The code expects event recordings and optional RPM-alignment CSV files on disk. Paths are supplied through the YAML config. The repository does not ship datasets.

## Citation

If you use this code, please cite:

```bibtex
@inproceedings{spetlik2026helixtrack,
  title     = {HelixTrack: Event-Based Tracking and RPM Estimation of Propeller-like Objects},
  author    = {Spetlik, Radim and Pliska, Michal and Vrba, Vojtech and Matas, Jiri},
  booktitle = {Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR), Findings},
  year      = {2026}
}
```
