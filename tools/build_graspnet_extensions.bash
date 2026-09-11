#!/usr/bin/env bash
set -eo pipefail
graspnet_project="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${graspnet_project}/tools/source_ubuntu_env.bash"
source "${graspnet_project}/tools/source_graspnet_build_env.bash"
set -u
cd "${graspnet_project}"

# Works for Git checkouts and verified source snapshots; --directory is relative
# to the main workspace, avoiding silently skipped patches in an ignored folder.
for graspnet_patch in \
    patches/graspnet-baseline/0001-use-fixed-width-knn-indices.patch \
    patches/graspnetAPI/0001-optional-evaluation-imports.patch \
    patches/graspnetAPI/0002-separate-inference-and-evaluation-dependencies.patch; do
    graspnet_package="$(basename "$(dirname "${graspnet_patch}")")"
    if ! git apply --reverse --check --directory="third_party/${graspnet_package}" "${graspnet_patch}" >/dev/null 2>&1; then
        git apply --check --directory="third_party/${graspnet_package}" "${graspnet_patch}"
        git apply --directory="third_party/${graspnet_package}" "${graspnet_patch}"
    fi
done
python -m pip install --no-build-isolation --no-deps \
    ./third_party/graspnet-baseline/pointnet2 \
    ./third_party/graspnet-baseline/knn \
    ./third_party/graspnetAPI
python -m pip install --no-build-isolation \
    -c src/rebotarm_vision/constraints-ubuntu.txt grasp-nms==1.0.2
python tools/check_local_perception.py
