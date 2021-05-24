#!/bin/bash
rm -r CMakeCache.txt
cmake . -D icl_core_DIR=~/workspace/gpu-voxels/build/packages/icl_core/ -D gpu_voxels_DIR=~/workspace/gpu-voxels/build/packages/gpu_voxels
make -j16
