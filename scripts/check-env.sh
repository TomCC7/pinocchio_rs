#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-2-Clause
#
# UT.1 — assert the pixi env exposes Pinocchio (+ Eigen, urdfdom, boost) via
# pkg-config. Fail fast with a readable message before any cargo build runs.

set -euo pipefail

# When called from `pixi run`, PKG_CONFIG_PATH is already set; otherwise we
# fall back to `pixi info --json` to find the env prefix.
if [[ -z "${PKG_CONFIG_PATH:-}" ]] && command -v pixi >/dev/null 2>&1; then
    PREFIX="$(pixi info --json 2>/dev/null | python3 -c \
        'import json,sys; d=json.load(sys.stdin); print(d["environments_info"][0]["prefix"])')"
    export PKG_CONFIG_PATH="${PREFIX}/lib/pkgconfig:${PREFIX}/share/pkgconfig"
fi

fail() {
    echo "ERROR: $*" >&2
    echo "Hint: run \`pixi install\` then re-run \`bash scripts/check-env.sh\`." >&2
    exit 1
}

require_pc() {
    local name="$1"
    if ! pkg-config --exists "${name}"; then
        fail "${name}.pc not found on PKG_CONFIG_PATH=${PKG_CONFIG_PATH:-<empty>}"
    fi
    local version
    version="$(pkg-config --modversion "${name}")"
    if [[ -z "${version}" ]]; then
        fail "${name}.pc found but reports an empty version"
    fi
    printf '  %-12s %s\n' "${name}" "${version}"
}

echo "pinocchio_rs env check:"
require_pc pinocchio
require_pc eigen3
require_pc urdfdom
echo "ok"
