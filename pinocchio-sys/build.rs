// SPDX-License-Identifier: BSD-2-Clause
//
// Build script for `pinocchio-sys`:
//   1. Locate the pixi env (preferring `pixi info --json`, then
//      `PIXI_PROJECT_ROOT`, finally trusting whatever `PKG_CONFIG_PATH` is
//      already set).
//   2. Probe `pkg-config` for `pinocchio` and `eigen3` to pick up include
//      dirs + link flags.
//   3. Configure `cxx_build` against the shim sources.

use std::path::{Path, PathBuf};
use std::process::Command;

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=shim/pinocchio_shim.h");
    println!("cargo:rerun-if-changed=shim/pinocchio_shim.cpp");
    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-env-changed=PIXI_PROJECT_ROOT");
    println!("cargo:rerun-if-env-changed=PKG_CONFIG_PATH");

    setup_pkg_config_path();

    let pinocchio = match pkg_config::Config::new()
        .cargo_metadata(true)
        .probe("pinocchio")
    {
        Ok(lib) => lib,
        Err(err) => fail_with_install_hint("pinocchio", err),
    };

    let eigen = match pkg_config::Config::new()
        .cargo_metadata(true)
        .probe("eigen3")
    {
        Ok(lib) => lib,
        Err(err) => fail_with_install_hint("eigen3", err),
    };

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

fn setup_pkg_config_path() {
    if let Some(prefix) = locate_pixi_prefix() {
        prepend_pkg_config_path(&prefix.join("lib").join("pkgconfig"));
        prepend_pkg_config_path(&prefix.join("share").join("pkgconfig"));
    }
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

fn fail_with_install_hint(pkg: &str, err: pkg_config::Error) -> ! {
    eprintln!();
    eprintln!("================================================================");
    eprintln!("pinocchio-sys: failed to locate `{pkg}` via pkg-config.");
    eprintln!();
    eprintln!("Underlying error: {err}");
    eprintln!();
    eprintln!("This crate expects native dependencies to come from a pixi env.");
    eprintln!("From the workspace root, run:");
    eprintln!();
    eprintln!("    pixi install");
    eprintln!("    pixi run cargo build");
    eprintln!();
    eprintln!("If you maintain Pinocchio outside pixi, set PKG_CONFIG_PATH to a");
    eprintln!("directory containing `pinocchio.pc` and `eigen3.pc` and re-run.");
    eprintln!("================================================================");
    std::process::exit(1);
}
