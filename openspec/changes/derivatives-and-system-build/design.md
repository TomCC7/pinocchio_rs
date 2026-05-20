## Context

Two distinct gaps surfaced after the MVP landed (commit `865142d`), both of them blocking a credible 0.2 release:

1. **Derivative coverage.** MVP exposes only `computeForwardKinematicsDerivatives` + `getJointVelocityDerivatives`. The reason this binding matters for control/optimization is the *full* derivative suite — RNEA derivatives (∂τ/∂q, ∂τ/∂v), ABA derivatives (∂q̈/∂q, ∂q̈/∂v), joint-acceleration partials, and per-frame derivative accessors. Without those, iLQR / DDP / MPC consumers must drop back to C++ Pinocchio.
2. **Build path is pixi-only by policy.** The current `build.rs` already probes `pkg-config` (so a system Pinocchio *can* be found), but it actively prepends the pixi env's `pkgconfig` directory and the failure message tells the user to `pixi install`. That makes pixi the only *blessed* path. Anyone on ROS-with-system-pinocchio, brew, or a bare conda env (without pixi) gets a worse experience than they need to, and `docs.rs` cannot render the API at all (it has no Pinocchio install and no network during builds).

This change closes both gaps. Neither touches the safe-API style (newtypes), the algorithm output model (D5 owned-copy for derivative matrices, matching §12), or the parity-test architecture (D6). It's additive on top of an already-stable foundation.

## Goals / Non-Goals

**Goals:**
- Bind RNEA derivatives, ABA derivatives, joint-acceleration derivatives, frame-velocity derivatives, and frame-acceleration derivatives. Cover them under the existing parity-test contract (`TOL_DERIV = 1e-8`).
- Make `cargo build` succeed against *any* `pkg-config`-discoverable Pinocchio without source edits. Keep `pixi run cargo build` working byte-identically.
- Make `cargo doc --features pinocchio-sys/docs-only` succeed in a network-free, Pinocchio-free environment so docs.rs can render.
- Extend CI to prove the system-install path (Ubuntu LTS via robotpkg) and the macOS pixi path actually work.

**Non-Goals:**
- New scalar types (`f32`, AD scalars) — explicitly deferred.
- Centroidal-dynamics derivatives (`dccrba`, `computeCentroidalMomentumDerivatives`) — the centroidal *forward* path isn't in MVP either; this belongs in a separate follow-up.
- Geometry / collision derivatives.
- Restructuring the goldens schema or tolerance constants.
- Windows support.

## Decisions

### DD1 — Derivative-API style: own the matrices, matching §12

**Decision.** Every new derivative accessor returns owned `DMatrix<f64>` (or a tuple of them), exactly like the existing `joint_velocity_derivatives` in `derivatives.rs:§12`. No `DMatrixView` flavor in this change.

**Rationale.** The §12 wrapper already standardized on owned copies because the typical consumer pattern (iLQR / DDP step) reads each derivative matrix once into its own per-step tensor — there is no inner loop reading the same matrix dozens of times. View-style accessors would also force a borrow on `&Data` that conflicts with the next derivative call's `&mut Data`. Owned copies are the right default; a future `*_view` sibling is a non-breaking additive change if benchmarks show the allocation is hot.

**Alternatives considered.** Borrowed `DMatrixView` for the large 6×nv blocks (rejected: ergonomic cost too high for negligible savings on Panda-sized models; revisit if a humanoid use case ever materializes). Caller-provided out-buffers (rejected: un-Rust-y; would be a duplicate API on top of the owned form).

### DD2 — RNEA's ∂τ/∂a equals the mass matrix — do not duplicate

**Decision.** `data.rnea_derivatives(...)` returns `(dtau_dq, dtau_dv)` only. The third partial (∂τ/∂a = M, the mass matrix) is **not** returned. Callers who want it use `data.mass_matrix(&model)`, which is already in §8.

**Rationale.** Returning the same matrix twice via two different APIs invites the "which one is the source of truth?" trap. CRBA fills M; RNEA-derivs internally calls CRBA to produce ∂τ/∂a. The wrapper docs say so explicitly. ABA-derivs analogously returns `(dddq_dq, dddq_dv)` and the third partial (∂q̈/∂τ = M⁻¹) is **not** exposed — callers cholesky-decompose `mass_matrix()` and invert.

**Alternatives considered.** Return all three partials per call (rejected: aliasing, duplicate work). Return ∂τ/∂a / ∂q̈/∂τ as views into Data (rejected: contradicts DD1 and risks the user mutating Data between the call and the read).

### DD3 — Cxx-bridge signature pattern: compute then accessor, mirroring §12

**Decision.** Each new derivative gets a *compute* function and one or more *accessor* functions, mirroring §12 and §9:

```cpp
void compute_rnea_derivatives(const Model&, Data&,
                              const double* q, std::size_t nq,
                              const double* v, std::size_t nv,
                              const double* a, std::size_t nv_a);

void data_rnea_derivatives(const Data&,
                           double* dtau_dq, double* dtau_dv,
                           std::size_t nv);
```

Same pattern for ABA-derivs, frame-velocity-derivs, frame-acceleration-derivs. Joint-acceleration-derivs reuses the existing `compute_forward_kinematics_derivatives` (Pinocchio populates everything in one pass) and only adds an accessor.

**Rationale.** Already proved out in §12. Keeps the unsafe boundary trivially auditable: each accessor writes one (or two) caller-owned column-major blocks of size 6×nv or nv×nv.

### DD4 — Build probe order: pkg-config first, pixi as automatic fallback

**Decision.** `build.rs` becomes:

```
1. Try pkg_config::probe("pinocchio") with PKG_CONFIG_PATH as the user set it.
2. If that fails, run `pixi info --json`; if a pixi env exists, prepend its
   pkgconfig dir and retry the probe.
3. If both fail, error out with a message that names *all* tried install paths
   (apt, brew, pixi, conda) and prints what PKG_CONFIG_PATH was.
4. On success, emit `cargo:warning=pinocchio-sys: found pinocchio <ver> at <path>`.
```

The current order is reversed: pixi is tried *first* and a system Pinocchio only matters if no pixi env exists. The new order respects the user's `PKG_CONFIG_PATH` first (least-surprise), and falls back to pixi automatically (so `pixi run cargo build` still works with zero ceremony).

**Rationale.** Today the build.rs already calls `pkg_config::probe`. The behaviour change is just "don't shadow the user's PKG_CONFIG_PATH unless asked." That's a one-function change to `setup_pkg_config_path`. The new diagnostic line is a 3-line addition. The whole "B half" of this change is ~50 LOC in `build.rs` plus tests; the *spec* and *CI* work is where the cost is.

**Alternatives considered.** Keep pixi-first as default (rejected: surprises consumers who installed Pinocchio system-wide; the resulting error "pkg-config can't find pinocchio" is misleading because pkg-config *could* find it but build.rs reshuffled the path). Make the order configurable via env var only, no Cargo feature (rejected: env-var-only config is non-discoverable and breaks `cargo doc` on docs.rs).

### DD5 — Cargo features as discovery overrides, not new code paths

**Decision.** Three features on `pinocchio-sys`:

| Feature | Effect | Who sets it |
|---------|--------|-------------|
| (none — default) | DD4 probe order: pkg-config first, pixi automatic fallback. | most consumers |
| `bundled-pixi` | DD4 step 2 (pixi prepend) runs *before* step 1 instead of after. Forces pixi env to win even if a system Pinocchio is also installed. | `pixi run` wrapper, set in `pixi.toml`'s cargo task |
| `system` | DD4 step 2 is skipped entirely. Probe fails immediately if PKG_CONFIG_PATH doesn't already point at Pinocchio. CI uses this to verify the system-install path actually works in isolation. | system-install CI job |
| `docs-only` | Skip `cxx_build` invocation entirely. Compile `src/lib.rs` with every extern function replaced by a stub returning `Default::default()` or panicking on call. | docs.rs build config |

**Rationale.** No business logic switches on these — they only steer the probe. Default behavior is the *new* "respect user, fall back to pixi" mode. Explicit features exist for the two CI lanes that need deterministic behavior, and for docs.rs.

`docs-only` cannot be the default for two obvious reasons: it produces a non-functional binary, and it would mask configuration mistakes. It must be opt-in.

**Alternatives considered.** Two binary features (`pixi` vs `system`) with no automatic fallback (rejected: noisy for the 90% case where users don't care). Env-var-driven (rejected: see DD4). A separate `pinocchio-sys-docs` crate (rejected: maintenance overhead for a single niche).

### DD6 — `docs-only` implementation: skip `cxx_build`, keep the bridge

**Decision (revised during implementation).** The `cxx::bridge` module is
present unconditionally. The `docs-only` feature only gates the *build script*
(skipping `pkg-config` probing and the `cxx_build` invocation). Rust source
compiles to `rmeta`/`rlib` containing `extern "C"` declarations that reference
`cxxbridge1$…` symbols, but those symbols never need to exist because
`cargo doc` does not link binaries — it generates documentation from the
compiled metadata.

```rust
// pinocchio-sys/build.rs
fn main() {
    // …rerun-if hooks…
    if cfg!(feature = "docs-only") {
        return;
    }
    // …pkg-config probe + cxx_build…
}
```

```rust
// pinocchio-sys/src/lib.rs — unchanged structure
#[cxx::bridge(namespace = "pinocchio_rs::shim")]
pub mod ffi { /* … declarations … */ }
```

**Rationale.** The originally planned `mod stubs` / `mod real` split hit
`cxx::UniquePtrTarget` immediately — that trait is implemented by the
`cxx::bridge` macro for opaque types, with bodies that call into the C++ ABI
glue. Hand-mirroring it for `UniquePtr<Model>` would have required
implementing six `#[doc(hidden)]` `unsafe` methods that all reference cxxbridge
externs, only to skip them at runtime. The simpler observation — that `cargo
doc` doesn't link, so unresolved externs are harmless — collapses the entire
problem to a one-line early-return in `build.rs`.

**Risk.** A consumer who attempts to *compile a binary or test* against
`pinocchio-sys` with `docs-only` enabled will get a linker error (undefined
references to `cxxbridge1$pinocchio_rs$shim$…`). This is the intended outcome
— the feature exists for documentation rendering only, and the rustdoc
"Features" section says so. CI's `cargo-doc-docs-only` job only runs `cargo
doc`, never `cargo build` or `cargo test` under `--features docs-only`.

**Alternatives reconsidered.** The hand-mirrored stub module remains a fallback
if `cxx` ever starts emitting checks at *compilation* time that require
`UniquePtrTarget` bodies to resolve. If that ever happens, the recovery is
mechanical (re-author `stubs.rs` with `unimplemented!()` bodies) and the spec
delta is captured in this DD.

### DD7 — `pixi.toml` cargo tasks switch to `--features bundled-pixi`

**Decision.** Update `pixi.toml`'s `cargo build` / `cargo test` / `gen-goldens` tasks to pass `--features pinocchio-sys/bundled-pixi`. That way the pixi-supported workflow is deterministic — it always uses the pixi env, never the system one.

**Rationale.** Without this, `pixi run cargo build` would *fall back* to pixi only if the system probe failed. That's mostly fine, but a contributor who happens to have a stale system Pinocchio installed would silently get the wrong one. Forcing the feature inside the pixi task removes the ambiguity.

### DD8 — CI matrix expansion

**Decision.** Three jobs in `.github/workflows/ci.yml`:

| Job | Runner | Install path | Gating |
|-----|--------|--------------|--------|
| `pixi-ubuntu` | ubuntu-latest | `pixi install` | gating (current MVP gate) |
| `pixi-macos` | macos-latest (arm64) | `pixi install` | non-gating initially, gating once stable |
| `system-ubuntu` | ubuntu-latest | `apt install robotpkg-pinocchio` via robotpkg PPA, no pixi; `--features pinocchio-sys/system` | non-gating initially |

Promotion of `pixi-macos` and `system-ubuntu` to gating is a follow-up after one fully-green week.

**Rationale.** Non-gating-first avoids landing the change in a state where flaky external repos block PRs. Robotpkg-pinocchio's apt index is unfortunately known to lag upstream; that's tolerable for a *smoke* signal (build + 2–3 quick tests) and intolerable for a *parity* gate, so the system-ubuntu job runs build + lib tests, not the full parity suite.

## Risks / Trade-offs

[Default probe order changes for non-pixi cargo users.] → Today, a user who has Pinocchio installed system-wide *and* a pixi env will get the pixi one. After this change, they get the system one. If their pixi env is the newer of the two, they'll silently downgrade. → **Mitigation:** the diagnostic line (`cargo:warning=pinocchio-sys: found pinocchio 3.9.0 at /opt/.../pinocchio.pc`) shows the chosen one explicitly. The `bundled-pixi` feature restores old behavior in one flag.

[`docs-only` stub drift.] → Hand-mirrored stub module diverges from the real bridge. → **Mitigation:** a `pinocchio-sys` unit test under `cfg(feature = "docs-only")` that imports every name from `ffi` and references it, so removing or renaming a function in the real bridge without updating stubs is a `cargo test --no-default-features --features docs-only` failure. If maintenance still proves too high, drop `docs-only` from this change and switch docs.rs to a `metadata.docs.rs` config that runs `apt install robotpkg-pinocchio` in the docs.rs sandbox — a documented escape hatch.

[Robotpkg apt index lag.] → `system-ubuntu` job uses a Pinocchio version that may be 1–2 minor versions behind the pixi-pinned one. → **Mitigation:** keep `system-ubuntu` non-gating, and constrain its test set to build + module-level unit tests (no goldens — those are pinned to the pixi version). The goal is "binding compiles and runs end-to-end against a system install," not "binding is parity-correct against arbitrary versions."

[ABA-derivs API symmetry with §12.] → ABA-derivs and RNEA-derivs both depend on FK having been computed; if the user calls them in the wrong order, Pinocchio reads stale Data internals. → **Mitigation:** the safe wrappers each call `forward_kinematics(...)` internally before invoking the derivative algorithm, so user-facing API is stateless modulo the (q, v, a/tau) inputs. Matches §12's `compute_forward_kinematics_derivatives` wrapper which already does this.

## Open Questions

- Do we ship `--features pinocchio-sys/system` as a transitive feature on `pinocchio-rs` (so consumers can `cargo build --features pinocchio-rs/system`)? Leaning yes — it's free to add and the most likely consumer-side knob. Decided during tasks step.
- Should `cargo:warning=` for the discovered pinocchio be downgraded to `cargo:rustc-cfg=...` (cleaner build output) or kept as a warning (more visible)? Leaning warning, since silent default-path changes are exactly what we want users to notice. Decide during tasks step.
