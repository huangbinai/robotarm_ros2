#!/usr/bin/env bash
# Source after tools/source_ubuntu_env.bash to build the pinned CUDA extensions.
# All toolchain files are project-local; this never updates the NVIDIA driver.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo "Source this file: source tools/source_graspnet_build_env.bash" >&2
    exit 2
fi
graspnet_build_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export CUDA_HOME="${graspnet_build_root}/third_party/cuda-12.1"
export CC="${graspnet_build_root}/third_party/gcc-12/usr/bin/gcc-12"
export CXX="${graspnet_build_root}/third_party/gcc-12/usr/bin/g++-12"
export CUDAHOSTCXX="${CXX}"
export TORCH_CUDA_ARCH_LIST="8.9"
export MAX_JOBS="1"
export PATH="${CUDA_HOME}/bin:${PATH}"
graspnet_include_dirs="${CUDA_HOME}/include"
for graspnet_include in "${graspnet_build_root}"/.venv-ros/lib/python3.12/site-packages/nvidia/*/include; do
    if [[ -d "${graspnet_include}" ]]; then
        graspnet_include_dirs="${graspnet_include_dirs}:${graspnet_include}"
    fi
done
export CPATH="${graspnet_include_dirs}${CPATH:+:${CPATH}}"
export LIBRARY_PATH="${CUDA_HOME}/lib${LIBRARY_PATH:+:${LIBRARY_PATH}}"
if [[ ! -x "${CUDA_HOME}/bin/nvcc" || ! -x "${CXX}" ]]; then
    echo "Missing project-local CUDA 12.1 / GCC 12 toolchain; see docs/local_ros_vision_zh.md" >&2
    return 1
fi
