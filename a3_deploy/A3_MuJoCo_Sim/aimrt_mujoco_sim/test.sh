#!/bin/bash

# exit on error and print each command
set -e

# cmake
cmake -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DAIMRT_MUJOCO_SIM_INSTALL=ON \
    -DCMAKE_INSTALL_PREFIX=./build/install \
    -DAIMRT_MUJOCO_SIM_BUILD_TESTS=ON \
    -DAIMRT_MUJOCO_SIM_BUILD_EXAMPLES=ON \
    $@

cmake --build build --config Release --parallel $(nproc)

cmake --build build --config Release --target test

