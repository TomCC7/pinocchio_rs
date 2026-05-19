// SPDX-License-Identifier: BSD-2-Clause
//! Pinocchio's `inverse-kinematics` example: damped least-squares IK targeting
//! a fixed SE(3) pose for `panda_hand`.

use nalgebra::{DMatrix, DVector};
use pinocchio_rs::{Data, Model, ReferenceFrame, SE3};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let model = Model::from_urdf(panda_urdf_path())?;
    let mut data = Data::new(&model);

    let frame_id = model.frame_id("panda_hand")?;

    // Pick a reachable target by sampling a random q and running FK on it,
    // then perturb the start config and try to recover the pose.
    let q_truth = model.random_configuration(0xCAFE);
    data.forward_kinematics(&model, &q_truth)?;
    data.update_frame_placements(&model);
    let target = data.frame_placement(frame_id);

    let mut q = model.neutral_configuration();
    let solved = solve_ik(&model, &mut data, frame_id, &target, &mut q)?;

    println!("converged in {solved} iterations");
    println!("final q   = {:.4?}", q.as_slice());
    Ok(())
}

pub(crate) fn solve_ik(
    model: &Model,
    data: &mut Data,
    frame_id: usize,
    target: &SE3,
    q: &mut DVector<f64>,
) -> Result<usize, Box<dyn std::error::Error>> {
    const TOL: f64 = 1e-4;
    const MAX_ITERS: usize = 200;
    const LAMBDA: f64 = 1e-6;
    const STEP: f64 = 1.0;

    for iter in 0..MAX_ITERS {
        data.forward_kinematics(model, q)?;
        data.update_frame_placements(model);
        let current = data.frame_placement(frame_id);

        // Error in the current frame's Local coords:
        //   err = log6(current^{-1} * target)
        let err = (current.inverse() * *target).log6();
        let err_norm = err.norm();
        if err_norm < TOL {
            return Ok(iter);
        }

        let j = data.frame_jacobian(model, q, frame_id, ReferenceFrame::Local)?;
        let nv = model.nv();

        // Damped LS:   dq = (Jᵀ J + λ² I)^{-1} Jᵀ err
        let jt = j.transpose();
        let mut a = &jt * &j;
        for i in 0..nv {
            a[(i, i)] += LAMBDA * LAMBDA;
        }
        let b: DVector<f64> = jt * err;
        let dq = a.lu().solve(&b).ok_or("LU solve failed")?;

        *q = model.integrate(q, &(dq * STEP))?;
        let _ = DMatrix::<f64>::zeros(0, 0); // keep the DMatrix use explicit
    }
    Err(format!("IK did not converge in {MAX_ITERS} iterations").into())
}

fn panda_urdf_path() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("data")
        .join("panda")
        .join("panda.urdf")
}
