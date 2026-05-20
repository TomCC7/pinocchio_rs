#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-2-Clause
#
# UT.13 — assert `pixi run gen-goldens` is byte-deterministic and produces all
# 8 expected files.

set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TMP1="$(mktemp -d)"
TMP2="$(mktemp -d)"
trap 'rm -rf "${TMP1}" "${TMP2}"' EXIT

BUILD_DIR="${ROOT}/tools/gen_goldens/build"
URDF="${ROOT}/pinocchio-rs/tests/data/panda/panda.urdf"

# Ensure binary exists (incremental rebuild OK).
cmake -S "${ROOT}/tools/gen_goldens" -B "${BUILD_DIR}" \
      -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build "${BUILD_DIR}" --parallel >/dev/null

"${BUILD_DIR}/gen_goldens" "${URDF}" "${TMP1}"
"${BUILD_DIR}/gen_goldens" "${URDF}" "${TMP2}"

expected_files=(
    fk_panda.json rnea_panda.json aba_panda.json crba_panda.json
    jac_panda.json se3_ops.json lie_ops.json fk_derivs_panda.json
    rnea_derivs_panda.json aba_derivs_panda.json
    frame_kinematics_derivs_panda.json
)
for f in "${expected_files[@]}"; do
    if [[ ! -f "${TMP1}/${f}" ]]; then
        echo "ERROR: expected file ${f} not produced" >&2
        exit 1
    fi
    if ! cmp -s "${TMP1}/${f}" "${TMP2}/${f}"; then
        echo "ERROR: ${f} differs between two consecutive runs" >&2
        exit 1
    fi
done

echo "goldens reproducibility check: OK (${#expected_files[@]}/${#expected_files[@]} files byte-identical)"
