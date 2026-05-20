// SPDX-License-Identifier: BSD-2-Clause
//
// C++ shim that bridges Pinocchio's templated headers to a non-templated,
// f64-only API consumable by Rust via `cxx`.
//
// We deliberately include the heavy Pinocchio headers here (not just a
// forward declaration) so that the cxx-generated TU sees `Model`/`Data` as
// complete types — `cxx` synthesises `unique_ptr<T>` plumbing that requires
// `sizeof(T)`.

#pragma once

#include "rust/cxx.h"

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>

namespace pinocchio_rs::shim {

// Concrete f64 instantiations (alias of `pinocchio::Model` / `Data`).
using Model = ::pinocchio::ModelTpl<double, 0, ::pinocchio::JointCollectionDefaultTpl>;
using Data = ::pinocchio::DataTpl<double, 0, ::pinocchio::JointCollectionDefaultTpl>;

// ---------------------------------------------------------------- version ---
::rust::String pinocchio_version();

// ---------------------------------------------------------------- model ----
std::unique_ptr<Model> build_model_from_urdf(::rust::Str path);
std::unique_ptr<Model> build_model_from_urdf_with_free_flyer(::rust::Str path);
std::unique_ptr<Model> build_model_from_urdf_string(::rust::Str xml);
std::unique_ptr<Model> build_model_from_urdf_string_with_free_flyer(::rust::Str xml);

std::size_t model_nq(const Model& model);
std::size_t model_nv(const Model& model);
std::size_t model_njoints(const Model& model);
std::size_t model_nframes(const Model& model);

std::size_t model_joint_id_by_name(const Model& model, ::rust::Str name);
std::size_t model_frame_id_by_name(const Model& model, ::rust::Str name);

bool model_has_joint(const Model& model, ::rust::Str name);
bool model_has_frame(const Model& model, ::rust::Str name);

::rust::String model_joint_name(const Model& model, std::size_t joint_id);

// ---------------------------------------------------------------- data ----
std::unique_ptr<Data> data_new(const Model& model);

// ---------------------------------------------------------------- SE3 -----
// All SE(3) ops take and return homogeneous 4×4 matrices in column-major
// layout (16 doubles). This trades a tiny copy for a stable ABI that
// avoids exposing Pinocchio's templated SE3Tpl<double, 0> across the FFI.

// log6(M) — the SE(3) logarithm, returned as a 6-vector
// (linear:3, angular:3) matching Pinocchio's `pinocchio::log6` convention.
void se3_log6(const double m_hom[16], double out6[6]);

// Jlog6(M) — the 6×6 Jacobian of log6 at M, column-major.
void se3_jlog6(const double m_hom[16], double out36[36]);

// ---------------------------------------------------------------- algorithms ---

// forwardKinematics(model, data, q)
void forward_kinematics(const Model& model, Data& data,
                        const double* q_ptr, std::size_t nq);
void forward_kinematics_v(const Model& model, Data& data,
                          const double* q_ptr, std::size_t nq,
                          const double* v_ptr, std::size_t nv);
void forward_kinematics_v_a(const Model& model, Data& data,
                            const double* q_ptr, std::size_t nq,
                            const double* v_ptr, std::size_t nv,
                            const double* a_ptr, std::size_t nv_a);

// updateFramePlacements(model, data) — needed before reading frame placements.
void update_frame_placements(const Model& model, Data& data);

// data.oMi[joint_id] -> 4×4 homogeneous (column-major, 16 doubles).
void data_joint_placement(const Data& data, std::size_t joint_id, double out16[16]);

// data.oMf[frame_id] -> 4×4 homogeneous; assumes update_frame_placements has run.
void data_frame_placement(const Data& data, std::size_t frame_id, double out16[16]);

// data.v[joint_id] -> [linear:3, angular:3] (6 doubles).
void data_joint_velocity(const Data& data, std::size_t joint_id, double out6[6]);

// Neutral configuration (length nq).
void model_neutral_configuration(const Model& model, double* out_q, std::size_t nq);

// ----- RNEA -----
void rnea(const Model& model, Data& data,
          const double* q_ptr, std::size_t nq,
          const double* v_ptr, std::size_t nv,
          const double* a_ptr, std::size_t nv_a);
void data_tau(const Data& data, double* out_tau, std::size_t nv);

// ----- ABA -----
void aba(const Model& model, Data& data,
         const double* q_ptr, std::size_t nq,
         const double* v_ptr, std::size_t nv,
         const double* tau_ptr, std::size_t nv_tau);
void data_ddq(const Data& data, double* out_ddq, std::size_t nv);

// ----- CRBA -----
void crba(const Model& model, Data& data,
          const double* q_ptr, std::size_t nq);
// Fill the lower triangle of data.M from the upper-triangular result of CRBA.
void data_mass_matrix_symmetrize(Data& data);
// Copy data.M into a caller-provided buffer (nv*nv doubles, column-major).
void data_mass_matrix_copy(const Data& data, double* out, std::size_t nv);

// ----- Jacobians -----
// `rf` encodes a ReferenceFrame: 0 = World, 1 = Local, 2 = LocalWorldAligned.
// The shim translates to pinocchio::ReferenceFrame internally.
void compute_joint_jacobian(const Model& model, Data& data,
                            const double* q_ptr, std::size_t nq,
                            std::size_t joint_id, double out6xnv[],
                            std::size_t nv);

void compute_frame_jacobian(const Model& model, Data& data,
                            const double* q_ptr, std::size_t nq,
                            std::size_t frame_id, std::uint8_t rf,
                            double out6xnv[], std::size_t nv);

// ----- Lie operations on the configuration manifold -----
void model_integrate(const Model& model,
                     const double* q_ptr, std::size_t nq,
                     const double* v_ptr, std::size_t nv,
                     double* out_q, std::size_t nq_out);
void model_difference(const Model& model,
                      const double* q0_ptr, const double* q1_ptr, std::size_t nq,
                      double* out_v, std::size_t nv_out);
void model_random_configuration(const Model& model, std::uint64_t seed,
                                double* out_q, std::size_t nq);

// ----- Forward-kinematics derivatives -----
void compute_forward_kinematics_derivatives(const Model& model, Data& data,
                                            const double* q_ptr, std::size_t nq,
                                            const double* v_ptr, std::size_t nv,
                                            const double* a_ptr, std::size_t nv_a);

// Pinocchio's getJointVelocityDerivatives writes both ∂v/∂q and ∂v/∂v at once.
// Each output is 6×nv, column-major. `rf` encodes ReferenceFrame as above.
void data_joint_velocity_derivatives(const Model& model, Data& data,
                                     std::size_t joint_id, std::uint8_t rf,
                                     double* dv_dq, double* dv_dv,
                                     std::size_t nv);

// ----- RNEA derivatives -----
// pinocchio::computeRNEADerivatives populates `data.dtau_dq`, `data.dtau_dv`,
// and `data.M` (the upper triangle of M). We expose only `dtau_dq` and
// `dtau_dv` — the third partial ∂τ/∂a = M and callers reach it via the CRBA
// path (`data_mass_matrix_*`).
void compute_rnea_derivatives(const Model& model, Data& data,
                              const double* q_ptr, std::size_t nq,
                              const double* v_ptr, std::size_t nv,
                              const double* a_ptr, std::size_t nv_a);
// Each output is nv×nv, column-major.
void data_rnea_derivatives(const Data& data,
                           double* dtau_dq, double* dtau_dv,
                           std::size_t nv);

// ----- ABA derivatives -----
// pinocchio::computeABADerivatives populates `data.ddq_dq`, `data.ddq_dv`,
// and `data.Minv` (the inverse mass matrix; we don't expose this — callers
// invert `mass_matrix()` themselves).
void compute_aba_derivatives(const Model& model, Data& data,
                             const double* q_ptr, std::size_t nq,
                             const double* v_ptr, std::size_t nv,
                             const double* tau_ptr, std::size_t nv_tau);
// Each output is nv×nv, column-major.
void data_aba_derivatives(const Data& data,
                          double* dddq_dq, double* dddq_dv,
                          std::size_t nv);

// ----- Joint-acceleration derivatives -----
// Reuses Pinocchio's `getJointAccelerationDerivatives`, which writes:
//   v_partial_dq, a_partial_dq, a_partial_dv, a_partial_da
// We expose only the three acceleration partials (the velocity ∂v/∂q is
// already covered by `data_joint_velocity_derivatives`).
// Requires `compute_forward_kinematics_derivatives` to have been called first.
// Each output is 6×nv, column-major.
void data_joint_acceleration_derivatives(const Model& model, Data& data,
                                         std::size_t joint_id, std::uint8_t rf,
                                         double* da_dq, double* da_dv,
                                         double* da_da,
                                         std::size_t nv);

// ----- Frame-velocity derivatives -----
// Wraps pinocchio::getFrameVelocityDerivatives. Each output is 6×nv,
// column-major. Requires `compute_forward_kinematics_derivatives` first.
void data_frame_velocity_derivatives(const Model& model, Data& data,
                                     std::size_t frame_id, std::uint8_t rf,
                                     double* dv_dq, double* dv_dv,
                                     std::size_t nv);

// ----- Frame-acceleration derivatives -----
// Wraps pinocchio::getFrameAccelerationDerivatives. Each output is 6×nv,
// column-major. Requires `compute_forward_kinematics_derivatives` first.
void data_frame_acceleration_derivatives(const Model& model, Data& data,
                                         std::size_t frame_id, std::uint8_t rf,
                                         double* da_dq, double* da_dv,
                                         double* da_da,
                                         std::size_t nv);

}  // namespace pinocchio_rs::shim
