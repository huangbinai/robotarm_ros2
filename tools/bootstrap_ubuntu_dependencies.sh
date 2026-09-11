#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
dependency_root="${REBOTARM_THIRD_PARTY_DIR:-${repo_root}/third_party}"

if ! command -v vcs >/dev/null 2>&1; then
    echo "vcstool is required: sudo apt install python3-vcstool" >&2
    exit 1
fi

mkdir -p "${dependency_root}"
vcs import "${dependency_root}" < "${repo_root}/rebotarm_dependencies.repos"

apply_managed_patch() {
    local repository="$1"
    local patch_file="$2"
    local repository_path="${dependency_root}/${repository}"
    local patch_path="${repo_root}/${patch_file}"

    if git -C "${repository_path}" apply --reverse --check "${patch_path}" >/dev/null 2>&1; then
        echo "patch already applied: ${repository}"
        return
    fi
    if ! git -C "${repository_path}" apply --check "${patch_path}"; then
        echo "patch does not apply cleanly: ${patch_file}" >&2
        exit 1
    fi
    git -C "${repository_path}" apply "${patch_path}"
    echo "patch applied: ${repository}"
}

apply_managed_patch \
    "graspnet-baseline" \
    "patches/graspnet-baseline/0001-use-fixed-width-knn-indices.patch"
apply_managed_patch \
    "graspnetAPI" \
    "patches/graspnetAPI/0001-optional-evaluation-imports.patch"
apply_managed_patch \
    "graspnetAPI" \
    "patches/graspnetAPI/0002-separate-inference-and-evaluation-dependencies.patch"

echo "Dependencies are ready in ${dependency_root}"
echo "Place GraspNet weights under ${repo_root}/models/graspnet and verify them with:"
echo "  cd ${repo_root} && sha256sum --check models/MANIFEST.sha256"
