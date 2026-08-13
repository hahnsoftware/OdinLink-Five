#!/usr/bin/env bash
# Build RCCL for Strix Halo with the compiler that avoids the ROCm 7.2.x
# first-collective regression. The installed ROCm runtime is not modified.
set -euo pipefail

RCCL_SRC="${RCCL_SRC:-${1:-/root/rccl-src}}"
LLVM_ROOT="${LLVM_ROOT:-/root/llvm-7.2.0/opt/rocm-7.2.0/llvm}"
JOBS="${JOBS:-$(nproc)}"

if [[ ! -x "$LLVM_ROOT/bin/amdclang++" ]]; then
    echo "Missing ROCm 7.2.0 compiler: $LLVM_ROOT/bin/amdclang++" >&2
    exit 1
fi
if [[ ! -x "$RCCL_SRC/install.sh" ]]; then
    echo "Missing RCCL source tree: $RCCL_SRC" >&2
    exit 1
fi

export CC="$LLVM_ROOT/bin/amdclang"
export CXX="$LLVM_ROOT/bin/amdclang++"
export CFLAGS="--rocm-path=/opt/rocm ${CFLAGS:-}"
export CXXFLAGS="--rocm-path=/opt/rocm ${CXXFLAGS:-}"

args=(--amdgpu_targets gfx1151 --disable-colltrace -j "$JOBS")
if [[ "${RCCL_NO_CLEAN:-0}" != 0 ]]; then
    args+=(--no_clean)
fi

cd "$RCCL_SRC"
./install.sh "${args[@]}"

sha256sum build/release/librccl.so.1.0