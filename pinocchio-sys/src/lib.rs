// SPDX-License-Identifier: BSD-2-Clause
//! Low-level FFI bindings to Pinocchio's C++ API.
//!
//! This crate is not meant for direct use — prefer the safe wrappers in
//! `pinocchio-rs`. Every function exposed here is a deliberate, hand-written
//! `cxx::bridge` entry; we do not auto-generate against Pinocchio's templated
//! headers.

#[cxx::bridge(namespace = "pinocchio_rs::shim")]
pub mod ffi {
    unsafe extern "C++" {
        include!("pinocchio-sys/shim/pinocchio_shim.h");

        /// Opaque alias for `pinocchio::Model` (f64 instantiation).
        type Model;

        /// Opaque alias for `pinocchio::Data` (f64 instantiation).
        type Data;

        /// Upstream Pinocchio version string (e.g. `"3.9.0"`).
        fn pinocchio_version() -> String;

        // ---- model construction ----
        fn build_model_from_urdf(path: &str) -> Result<UniquePtr<Model>>;
        fn build_model_from_urdf_with_free_flyer(path: &str) -> Result<UniquePtr<Model>>;
        fn build_model_from_urdf_string(xml: &str) -> Result<UniquePtr<Model>>;
        fn build_model_from_urdf_string_with_free_flyer(xml: &str)
            -> Result<UniquePtr<Model>>;

        // ---- model accessors ----
        fn model_nq(model: &Model) -> usize;
        fn model_nv(model: &Model) -> usize;
        fn model_njoints(model: &Model) -> usize;
        fn model_nframes(model: &Model) -> usize;

        /// Returns `njoints` if `name` is not a joint (caller checks bounds).
        fn model_joint_id_by_name(model: &Model, name: &str) -> usize;
        /// Returns `nframes` if `name` is not a frame (caller checks bounds).
        fn model_frame_id_by_name(model: &Model, name: &str) -> usize;

        fn model_has_joint(model: &Model, name: &str) -> bool;
        fn model_has_frame(model: &Model, name: &str) -> bool;

        fn model_joint_name(model: &Model, joint_id: usize) -> Result<String>;

        // ---- data construction ----
        fn data_new(model: &Model) -> UniquePtr<Data>;

        // ---- SE(3) ops ----
        // Inputs/outputs are column-major flat buffers — see shim header.
        unsafe fn se3_log6(m_hom: *const f64, out6: *mut f64);
        unsafe fn se3_jlog6(m_hom: *const f64, out36: *mut f64);

        // ---- Forward kinematics ----
        unsafe fn forward_kinematics(
            model: &Model, data: Pin<&mut Data>,
            q_ptr: *const f64, nq: usize,
        );
        unsafe fn forward_kinematics_v(
            model: &Model, data: Pin<&mut Data>,
            q_ptr: *const f64, nq: usize,
            v_ptr: *const f64, nv: usize,
        );
        unsafe fn forward_kinematics_v_a(
            model: &Model, data: Pin<&mut Data>,
            q_ptr: *const f64, nq: usize,
            v_ptr: *const f64, nv: usize,
            a_ptr: *const f64, nv_a: usize,
        );
        fn update_frame_placements(model: &Model, data: Pin<&mut Data>);

        unsafe fn data_joint_placement(data: &Data, joint_id: usize, out16: *mut f64);
        unsafe fn data_frame_placement(data: &Data, frame_id: usize, out16: *mut f64);
        unsafe fn data_joint_velocity(data: &Data, joint_id: usize, out6: *mut f64);

        unsafe fn model_neutral_configuration(model: &Model, out_q: *mut f64, nq: usize);

        // ---- RNEA ----
        unsafe fn rnea(
            model: &Model, data: Pin<&mut Data>,
            q_ptr: *const f64, nq: usize,
            v_ptr: *const f64, nv: usize,
            a_ptr: *const f64, nv_a: usize,
        );
        unsafe fn data_tau(data: &Data, out_tau: *mut f64, nv: usize);

        // ---- ABA ----
        unsafe fn aba(
            model: &Model, data: Pin<&mut Data>,
            q_ptr: *const f64, nq: usize,
            v_ptr: *const f64, nv: usize,
            tau_ptr: *const f64, nv_tau: usize,
        );
        unsafe fn data_ddq(data: &Data, out_ddq: *mut f64, nv: usize);

        // ---- CRBA ----
        unsafe fn crba(
            model: &Model, data: Pin<&mut Data>,
            q_ptr: *const f64, nq: usize,
        );
        fn data_mass_matrix_symmetrize(data: Pin<&mut Data>);
        unsafe fn data_mass_matrix_copy(data: &Data, out: *mut f64, nv: usize);

        // ---- Jacobians ----
        // `rf` encodes a ReferenceFrame: 0 = World, 1 = Local, 2 = LocalWorldAligned.
        unsafe fn compute_joint_jacobian(
            model: &Model, data: Pin<&mut Data>,
            q_ptr: *const f64, nq: usize,
            joint_id: usize, out6xnv: *mut f64, nv: usize,
        );
        unsafe fn compute_frame_jacobian(
            model: &Model, data: Pin<&mut Data>,
            q_ptr: *const f64, nq: usize,
            frame_id: usize, rf: u8, out6xnv: *mut f64, nv: usize,
        );

        // ---- Lie operations ----
        unsafe fn model_integrate(
            model: &Model,
            q_ptr: *const f64, nq: usize,
            v_ptr: *const f64, nv: usize,
            out_q: *mut f64, nq_out: usize,
        );
        unsafe fn model_difference(
            model: &Model,
            q0_ptr: *const f64, q1_ptr: *const f64, nq: usize,
            out_v: *mut f64, nv_out: usize,
        );
        unsafe fn model_random_configuration(
            model: &Model, seed: u64,
            out_q: *mut f64, nq: usize,
        );

        // ---- Forward-kinematics derivatives ----
        unsafe fn compute_forward_kinematics_derivatives(
            model: &Model, data: Pin<&mut Data>,
            q_ptr: *const f64, nq: usize,
            v_ptr: *const f64, nv: usize,
            a_ptr: *const f64, nv_a: usize,
        );
        unsafe fn data_joint_velocity_derivatives(
            model: &Model, data: Pin<&mut Data>,
            joint_id: usize, rf: u8,
            dv_dq: *mut f64, dv_dv: *mut f64,
            nv: usize,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::ffi;

    /// UT.2 — Pinocchio's version string must be non-empty, contain a digit,
    /// and match the major.minor we expect from `pixi.toml`.
    #[test]
    fn pinocchio_version_is_well_formed() {
        let v = ffi::pinocchio_version();
        assert!(!v.is_empty(), "pinocchio_version() returned an empty string");
        assert!(
            v.chars().any(|c| c.is_ascii_digit()),
            "pinocchio_version()={v:?} contains no digit",
        );

        let major = v.split('.').next().expect("version has a major component");
        assert_eq!(
            major, "3",
            "expected Pinocchio 3.x (pixi.toml pins >=3.2,<4) but got {v:?}",
        );
    }

    /// UT.3a — building from an in-memory URDF string returns a usable model
    /// with `nq == 7` for a 7-revolute-joint arm.
    #[test]
    fn build_model_from_minimal_urdf_string() {
        // Minimal 7-revolute-joint serial chain. Each joint has identity
        // origin / unit z-axis / no inertia — enough to exercise the parser
        // path. `nq == nv == 7` because all joints are revolute and no
        // free-flyer is added.
        let xml = minimal_7r_urdf();
        let model = ffi::build_model_from_urdf_string(&xml)
            .expect("urdf string parses");
        assert!(!model.is_null(), "build returned a null UniquePtr<Model>");
        let m = model.as_ref().expect("non-null model");
        assert_eq!(ffi::model_nq(m), 7, "minimal 7R chain has nq == 7");
        assert_eq!(ffi::model_nv(m), 7, "minimal 7R chain has nv == 7");
        // njoints includes the implicit universe joint at index 0.
        assert_eq!(ffi::model_njoints(m), 8);

        // Name → id → name round-trip for joint #1.
        let id = ffi::model_joint_id_by_name(m, "j1");
        assert_eq!(id, 1, "j1 is the first non-universe joint");
        let name = ffi::model_joint_name(m, id).expect("valid id");
        assert_eq!(name, "j1");

        // Out-of-range joint id surfaces as a Result::Err (cxx exception).
        let oor = ffi::model_joint_name(m, 999);
        assert!(oor.is_err(), "out-of-range joint id must error");

        // Unknown name returns the njoints sentinel.
        let missing = ffi::model_joint_id_by_name(m, "nonexistent");
        assert_eq!(missing, ffi::model_njoints(m));
    }

    /// Bad XML must propagate as `Result::Err`.
    #[test]
    fn build_model_from_garbage_xml_errors() {
        let result = ffi::build_model_from_urdf_string("<not a urdf at all>");
        assert!(result.is_err(), "malformed URDF must produce an Err");
    }

    /// UT.3c (shim-side) — Data construction from a freshly built model.
    #[test]
    fn data_new_succeeds() {
        let model = ffi::build_model_from_urdf_string(&minimal_7r_urdf()).unwrap();
        let data = ffi::data_new(model.as_ref().unwrap());
        assert!(!data.is_null(), "Data::new returned a null UniquePtr<Data>");
    }

    fn minimal_7r_urdf() -> String {
        let mut s = String::from(
            r#"<?xml version="1.0"?>
<robot name="minimal7r">
  <link name="base"/>
"#,
        );
        for i in 1..=7 {
            s.push_str(&format!(
                r#"  <link name="l{i}"/>
  <joint name="j{i}" type="revolute">
    <parent link="{parent}"/>
    <child link="l{i}"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="10" velocity="1" lower="-3.14" upper="3.14"/>
  </joint>
"#,
                i = i,
                parent = if i == 1 { "base".to_string() } else { format!("l{}", i - 1) },
            ));
        }
        s.push_str("</robot>\n");
        s
    }
}
