// SPDX-License-Identifier: BSD-2-Clause
//
// Build script for `pinocchio-sys`.
//
// Discovery order (see also crate-level rustdoc / design DD4):
//
//   * Default — probe `pkg-config` for `pinocchio` using whatever
//     `PKG_CONFIG_PATH` the user has set. If that fails, fall back to
//     discovering a pixi env (`pixi info --json` → `PIXI_PROJECT_ROOT`)
//     and retry.
//   * `bundled-pixi` feature — prepend the pixi env's pkgconfig dir BEFORE
//     the first probe attempt. The pixi env wins even when a system
//     Pinocchio is also installed.
//   * `system` feature — pixi auto-discovery is disabled entirely; the
//     probe must succeed against the user's PKG_CONFIG_PATH.
//   * `docs-only` feature — skip the entire native build flow; the crate
//     compiles with stub bodies that panic on call.
//
// On a successful probe, we emit exactly one `cargo:warning=` line naming
// the Pinocchio version and the directory containing `pinocchio.pc`, so
// the chosen install is visible in cargo's build output.

use std::path::{Path, PathBuf};
use std::process::Command;

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=shim/pinocchio_shim.h");
    println!("cargo:rerun-if-changed=shim/pinocchio_shim.cpp");
    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-env-changed=PIXI_PROJECT_ROOT");
    println!("cargo:rerun-if-env-changed=PKG_CONFIG_PATH");

    // docs-only — no native compilation, no pkg-config probe, no cxx_build.
    if cfg!(feature = "docs-only") {
        return;
    }

    let pkg_config_path_initial = std::env::var("PKG_CONFIG_PATH").unwrap_or_default();
    let mut tried_pixi_prefix: Option<PathBuf> = None;

    // bundled-pixi: prepend pixi paths before the first probe attempt.
    if cfg!(feature = "bundled-pixi") {
        tried_pixi_prefix = prepend_pixi_paths();
    }

    let (pinocchio, eigen) = match probe_pair() {
        Ok(pair) => pair,
        Err(first_err) => {
            // `system` feature → no automatic pixi fallback.
            if cfg!(feature = "system") {
                fail_with_install_hint(first_err, &pkg_config_path_initial, None);
            }
            // bundled-pixi already prepended the pixi env and the probe still
            // failed — nothing left to try.
            if cfg!(feature = "bundled-pixi") {
                fail_with_install_hint(first_err, &pkg_config_path_initial, tried_pixi_prefix);
            }
            // Default behaviour: try the pixi fallback now and retry.
            tried_pixi_prefix = prepend_pixi_paths();
            if tried_pixi_prefix.is_none() {
                fail_with_install_hint(first_err, &pkg_config_path_initial, None);
            }
            match probe_pair() {
                Ok(pair) => pair,
                Err(second_err) => {
                    fail_with_install_hint(second_err, &pkg_config_path_initial, tried_pixi_prefix)
                }
            }
        }
    };

    emit_found_diagnostic(&pinocchio);

    // pkg-config's `cargo_metadata(true)` emits `rustc-link-search` so the
    // *linker* finds libpinocchio.so / .dylib, but it does NOT emit an rpath,
    // which is what the dynamic loader needs at *runtime*. Linux happens to
    // work inside `pixi run` because pixi activation sets LD_LIBRARY_PATH;
    // macOS does not honor LD_LIBRARY_PATH and dyld instead looks at the
    // binary's embedded rpath list. Without this we get
    // `dyld: Library not loaded: @rpath/libpinocchio_default.dylib`
    // when running `cargo test` (or any built artifact) on macOS, and the
    // same class of failure on Linux outside a pixi-activated shell. Mirror
    // Pinocchio's own `pinocchio.pc` link dir into the binary so it works
    // regardless of how the host env is set up.
    for link_path in pinocchio.link_paths.iter().chain(eigen.link_paths.iter()) {
        println!("cargo:rustc-link-arg=-Wl,-rpath,{}", link_path.display());
    }

    let mut build = cxx_build::bridge("src/lib.rs");
    build
        .file("shim/pinocchio_shim.cpp")
        .std("c++17")
        .flag_if_supported("-Wno-deprecated-declarations")
        .flag_if_supported("-Wno-unused-parameter")
        .flag_if_supported("-Wno-pessimizing-move");

    // Pinocchio is templated and likes `-DPINOCCHIO_WITH_URDFDOM` etc.; we
    // inherit whatever flags `pkg-config --cflags` advertises rather than
    // hard-code defines.
    for inc in pinocchio.include_paths.iter().chain(eigen.include_paths.iter()) {
        build.include(inc);
    }
    // Ensure our own header is found via "pinocchio-sys/shim/...".
    let manifest_dir = PathBuf::from(env_var("CARGO_MANIFEST_DIR"));
    build.include(manifest_dir.parent().expect("workspace root above pinocchio-sys"));

    build.compile("pinocchio_shim");
}

/// Probe both `pinocchio` and `eigen3` via pkg-config, returning the libraries
/// on success. The pinocchio probe is attempted first so its error surfaces
/// first in the diagnostic.
fn probe_pair() -> Result<(pkg_config::Library, pkg_config::Library), pkg_config::Error> {
    let pinocchio = pkg_config::Config::new()
        .cargo_metadata(true)
        .probe("pinocchio")?;
    let eigen = pkg_config::Config::new()
        .cargo_metadata(true)
        .probe("eigen3")?;
    Ok((pinocchio, eigen))
}

/// Locate a pixi env via `pixi info --json` or `PIXI_PROJECT_ROOT` and prepend
/// its `lib/pkgconfig` + `share/pkgconfig` directories to `PKG_CONFIG_PATH`.
/// Returns the pixi prefix that was used, if any.
fn prepend_pixi_paths() -> Option<PathBuf> {
    let prefix = locate_pixi_prefix()?;
    prepend_pkg_config_path(&prefix.join("lib").join("pkgconfig"));
    prepend_pkg_config_path(&prefix.join("share").join("pkgconfig"));
    Some(prefix)
}

fn locate_pixi_prefix() -> Option<PathBuf> {
    if let Ok(out) = Command::new("pixi").args(["info", "--json"]).output() {
        if out.status.success() {
            if let Some(prefix) = parse_pixi_prefix(&String::from_utf8_lossy(&out.stdout)) {
                return Some(prefix);
            }
        }
    }
    if let Ok(root) = std::env::var("PIXI_PROJECT_ROOT") {
        let candidate = Path::new(&root).join(".pixi").join("envs").join("default");
        if candidate.exists() {
            return Some(candidate);
        }
    }
    None
}

fn parse_pixi_prefix(json: &str) -> Option<PathBuf> {
    // Tiny ad-hoc extractor — avoids pulling serde_json into the build deps.
    // Looks for the first `"prefix": "<path>"` occurrence.
    let key = "\"prefix\"";
    let idx = json.find(key)?;
    let after = &json[idx + key.len()..];
    let colon = after.find(':')?;
    let rest = &after[colon + 1..];
    let start = rest.find('"')?;
    let after_quote = &rest[start + 1..];
    let end = after_quote.find('"')?;
    Some(PathBuf::from(&after_quote[..end]))
}

fn prepend_pkg_config_path(dir: &Path) {
    if !dir.exists() {
        return;
    }
    let existing = std::env::var_os("PKG_CONFIG_PATH").unwrap_or_default();
    let mut new = std::ffi::OsString::from(dir);
    if !existing.is_empty() {
        new.push(":");
        new.push(&existing);
    }
    std::env::set_var("PKG_CONFIG_PATH", &new);
}

fn env_var(name: &str) -> String {
    std::env::var(name).unwrap_or_else(|_| panic!("env var {name} not set"))
}

/// Emit a `cargo:warning=...` line naming the discovered Pinocchio version
/// and the directory containing `pinocchio.pc`. Visible by default in cargo's
/// build output, so misconfigured environments are immediately debuggable.
fn emit_found_diagnostic(pinocchio: &pkg_config::Library) {
    let pcfiledir = Command::new("pkg-config")
        .args(["--variable=pcfiledir", "pinocchio"])
        .output()
        .ok()
        .and_then(|out| {
            if out.status.success() {
                Some(String::from_utf8_lossy(&out.stdout).trim().to_string())
            } else {
                None
            }
        })
        .unwrap_or_else(|| "(unknown path)".to_string());

    println!(
        "cargo:warning=pinocchio-sys: found pinocchio {} at {}/pinocchio.pc",
        pinocchio.version, pcfiledir,
    );
}

/// Print a multi-path diagnostic listing every discovery path tried and the
/// four documented install options, then exit non-zero.
fn fail_with_install_hint(
    err: pkg_config::Error,
    pkg_config_path: &str,
    tried_pixi: Option<PathBuf>,
) -> ! {
    eprintln!();
    eprintln!("================================================================");
    eprintln!("pinocchio-sys: failed to locate Pinocchio (or its dependencies)");
    eprintln!("via pkg-config.");
    eprintln!();
    eprintln!("Underlying pkg-config error:");
    eprintln!("  {err}");
    eprintln!();
    eprintln!("Probe attempted with PKG_CONFIG_PATH =");
    let final_path = std::env::var("PKG_CONFIG_PATH").unwrap_or_default();
    let pkg_config_path = if pkg_config_path.is_empty() { "(empty)" } else { pkg_config_path };
    let final_path_display = if final_path.is_empty() { "(empty)".to_string() } else { final_path };
    eprintln!("  initial: {pkg_config_path}");
    eprintln!("  final  : {final_path_display}");
    if let Some(prefix) = tried_pixi {
        eprintln!();
        eprintln!("Pixi auto-discovery ran and yielded prefix:");
        eprintln!("  {}", prefix.display());
        eprintln!("(its `lib/pkgconfig` was prepended above, but the probe still failed.)");
    } else if cfg!(feature = "system") {
        eprintln!();
        eprintln!("Pixi auto-discovery was disabled by the `system` Cargo feature.");
    } else {
        eprintln!();
        eprintln!("Pixi auto-discovery did not find a pixi env (`pixi info --json`");
        eprintln!("had no usable `prefix` and `PIXI_PROJECT_ROOT` was unset or invalid).");
    }
    eprintln!();
    eprintln!("To resolve, install Pinocchio via ONE of:");
    eprintln!();
    eprintln!("  1. pixi (recommended for development of this crate)");
    eprintln!("       pixi install");
    eprintln!("       pixi run cargo build");
    eprintln!();
    eprintln!("  2. apt + robotpkg PPA (Linux consumers, system-wide)");
    eprintln!("       see http://robotpkg.openrobots.org/install.html");
    eprintln!("       sudo apt install robotpkg-pinocchio libeigen3-dev");
    eprintln!();
    eprintln!("  3. homebrew (macOS consumers, system-wide)");
    eprintln!("       brew install pinocchio eigen");
    eprintln!();
    eprintln!("  4. conda (without pixi)");
    eprintln!("       conda install -c conda-forge pinocchio eigen urdfdom");
    eprintln!();
    eprintln!("Then re-run `cargo build`. If you installed Pinocchio outside the");
    eprintln!("default search path, set PKG_CONFIG_PATH to a directory containing");
    eprintln!("`pinocchio.pc` and `eigen3.pc`.");
    eprintln!("================================================================");
    std::process::exit(1);
}
