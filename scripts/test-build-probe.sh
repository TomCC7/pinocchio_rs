#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-2-Clause
#
# UT.2b — Outside `pixi run`, with PKG_CONFIG_PATH unset, the pinocchio-sys
# build script SHOULD discover the pixi env automatically and emit a
# `cargo:warning=pinocchio-sys: found pinocchio … at <pixi-env-path>` line.
#
# Proves the automatic pixi fallback is intact even when the user never
# activates the pixi env. Assumes no system Pinocchio in the default
# pkg-config search path (which is the normal dev/CI shape — the
# `system-ubuntu` CI job, which installs Pinocchio system-wide, runs with
# `--features system` and does NOT exercise this script).

set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TMP_TARGET="$(mktemp -d)"
LOG="$(mktemp)"
trap 'rm -rf "${TMP_TARGET}"; rm -f "${LOG}"' EXIT

# Locate cargo. Inside `pixi run`, `cargo` is on PATH via the pixi env. We
# want the same binary but invoked WITHOUT pixi activation so the build
# script's own discovery code is the only thing populating PKG_CONFIG_PATH.
CARGO_BIN="$(command -v cargo)"

# Run cargo build with a fresh target dir, no PKG_CONFIG_PATH inherited.
# `cargo build` may fail downstream (e.g., link), but the build script
# diagnostic line should appear in stderr regardless because it is emitted
# before cxx_build runs.
env -u PKG_CONFIG_PATH \
    CARGO_TARGET_DIR="${TMP_TARGET}" \
    "${CARGO_BIN}" build -p pinocchio-sys --manifest-path "${ROOT}/Cargo.toml" \
    >"${LOG}" 2>&1 || true

if grep -E "warning: .*pinocchio-sys: found pinocchio .* at .*/\.pixi/envs/" "${LOG}" >/dev/null; then
    echo "build-probe: OK — pixi fallback emitted expected diagnostic"
    exit 0
fi

# If the system pkg-config search path happens to contain a pinocchio.pc,
# the fallback never fires and the test is inapplicable — surface that
# clearly rather than silently failing.
if grep -E "warning: .*pinocchio-sys: found pinocchio" "${LOG}" >/dev/null; then
    echo "build-probe: SKIP — a non-pixi Pinocchio was found first:"
    grep -E "warning: .*pinocchio-sys: found pinocchio" "${LOG}"
    echo "This script only meaningfully runs on a host without system Pinocchio."
    exit 0
fi

echo "build-probe: FAIL — no 'pinocchio-sys: found pinocchio …' warning observed."
echo "Log tail:"
tail -60 "${LOG}"
exit 1
