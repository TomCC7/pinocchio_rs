// SPDX-License-Identifier: BSD-2-Clause
//! UT.11 — IK convergence on 10 reachable random targets on the Panda model.

use nalgebra::DVector;
use pinocchio_rs::{Data, Model, ReferenceFrame, SE3};

#[test]
fn ik_converges_on_reachable_targets() {
    let model = Model::from_urdf(panda_urdf_path()).expect("load urdf");
    let mut data = Data::new(&model);
    let frame_id = model.frame_id("panda_hand").expect("frame exists");

    let mut successes = 0;
    for seed in 0u64..10 {
        // Generate a reachable pose by forward-kinematics on a random q_truth,
        // then start from a perturbed (or neutral) seed config.
        let q_truth = model.random_configuration(0x1000 + seed);
        data.forward_kinematics(&model, &q_truth).unwrap();
        data.update_frame_placements(&model);
        let target = data.frame_placement(frame_id);

        let mut q = model.neutral_configuration();
        match solve_ik(&model, &mut data, frame_id, &target, &mut q, seed) {
            Ok(iters) => {
                assert!(iters < 200, "seed {seed}: too many iters ({iters})");
                successes += 1;
            }
            Err(e) => panic!("seed {seed}: IK failed: {e}"),
        }
    }
    assert_eq!(successes, 10);
}

fn solve_ik(
    model: &Model,
    data: &mut Data,
    frame_id: usize,
    target: &SE3,
    q: &mut DVector<f64>,
    _seed: u64,
) -> Result<usize, String> {
    const TOL: f64 = 1e-4;
    const MAX_ITERS: usize = 200;
    const LAMBDA: f64 = 1e-6;

    for iter in 0..MAX_ITERS {
        data.forward_kinematics(model, q).map_err(|e| e.to_string())?;
        data.update_frame_placements(model);
        let current = data.frame_placement(frame_id);

        let err = (current.inverse() * *target).log6();
        if err.norm() < TOL {
            return Ok(iter);
        }
        let j = data
            .frame_jacobian(model, q, frame_id, ReferenceFrame::Local)
            .map_err(|e| e.to_string())?;
        let nv = model.nv();
        let jt = j.transpose();
        let mut a = &jt * &j;
        for i in 0..nv {
            a[(i, i)] += LAMBDA * LAMBDA;
        }
        let b = jt * err;
        let dq = a.lu().solve(&b).ok_or("LU solve failed")?;
        *q = model.integrate(q, &dq).map_err(|e| e.to_string())?;
    }
    Err(format!("did not converge in {MAX_ITERS} iters"))
}

fn panda_urdf_path() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("data")
        .join("panda")
        .join("panda.urdf")
}
