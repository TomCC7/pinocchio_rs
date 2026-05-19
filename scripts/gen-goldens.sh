#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-2-Clause
#
# Build the gen_goldens C++ oracle binary against the pixi env's Pinocchio and
# run it against the vendored Panda URDF, writing JSONs into
# `pinocchio-rs/tests/goldens/`.

set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="${ROOT}/tools/gen_goldens/build"
SRC_DIR="${ROOT}/tools/gen_goldens"
URDF="${ROOT}/pinocchio-rs/tests/data/panda/panda.urdf"
OUT_DIR="${ROOT}/pinocchio-rs/tests/goldens"

mkdir -p "${BUILD_DIR}" "${OUT_DIR}"

cmake -S "${SRC_DIR}" -B "${BUILD_DIR}" \
      -DCMAKE_BUILD_TYPE=Release \
      >/dev/null
cmake --build "${BUILD_DIR}" --parallel >/dev/null

"${BUILD_DIR}/gen_goldens" "${URDF}" "${OUT_DIR}"
