// SPDX-License-Identifier: BSD-2-Clause

#include "pinocchio-sys/shim/pinocchio_shim.h"

#include <pinocchio/config.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/spatial/se3.hpp>

#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <Eigen/Core>

#include <cstdint>

#include <stdexcept>
#include <string>

namespace pinocchio_rs::shim {

::rust::String pinocchio_version() {
    return ::rust::String{PINOCCHIO_VERSION};
}

namespace {

inline std::string to_std_string(::rust::Str s) {
    return std::string{s.data(), s.size()};
}

}  // namespace

std::unique_ptr<Model> build_model_from_urdf(::rust::Str path) {
    auto model = std::make_unique<Model>();
    ::pinocchio::urdf::buildModel(to_std_string(path), *model);
    return model;
}

std::unique_ptr<Model> build_model_from_urdf_with_free_flyer(::rust::Str path) {
    auto model = std::make_unique<Model>();
    ::pinocchio::urdf::buildModel(
        to_std_string(path),
        ::pinocchio::JointModelFreeFlyer(),
        *model);
    return model;
}

std::unique_ptr<Model> build_model_from_urdf_string(::rust::Str xml) {
    auto model = std::make_unique<Model>();
    ::pinocchio::urdf::buildModelFromXML(to_std_string(xml), *model);
    return model;
}

std::unique_ptr<Model> build_model_from_urdf_string_with_free_flyer(::rust::Str xml) {
    auto model = std::make_unique<Model>();
    ::pinocchio::urdf::buildModelFromXML(
        to_std_string(xml),
        ::pinocchio::JointModelFreeFlyer(),
        *model);
    return model;
}

std::size_t model_nq(const Model& model) {
    return static_cast<std::size_t>(model.nq);
}

std::size_t model_nv(const Model& model) {
    return static_cast<std::size_t>(model.nv);
}

std::size_t model_njoints(const Model& model) {
    return static_cast<std::size_t>(model.njoints);
}

std::size_t model_nframes(const Model& model) {
    return static_cast<std::size_t>(model.nframes);
}

std::size_t model_joint_id_by_name(const Model& model, ::rust::Str name) {
    const auto s = to_std_string(name);
    if (!model.existJointName(s)) {
        return static_cast<std::size_t>(model.njoints);
    }
    return static_cast<std::size_t>(model.getJointId(s));
}

std::size_t model_frame_id_by_name(const Model& model, ::rust::Str name) {
    const auto s = to_std_string(name);
    if (!model.existFrame(s)) {
        return static_cast<std::size_t>(model.nframes);
    }
    return static_cast<std::size_t>(model.getFrameId(s));
}

bool model_has_joint(const Model& model, ::rust::Str name) {
    return model.existJointName(to_std_string(name));
}

bool model_has_frame(const Model& model, ::rust::Str name) {
    return model.existFrame(to_std_string(name));
}

::rust::String model_joint_name(const Model& model, std::size_t joint_id) {
    if (joint_id >= static_cast<std::size_t>(model.njoints)) {
        throw std::out_of_range(
            "model_joint_name: joint_id " + std::to_string(joint_id)
            + " >= njoints " + std::to_string(model.njoints));
    }
    return ::rust::String{model.names[joint_id]};
}

std::unique_ptr<Data> data_new(const Model& model) {
    return std::make_unique<Data>(model);
}

namespace {

using SE3 = ::pinocchio::SE3Tpl<double, 0>;
using Mat4Map = ::Eigen::Map<const ::Eigen::Matrix<double, 4, 4>>;
using Vec6Map = ::Eigen::Map<::Eigen::Matrix<double, 6, 1>>;
using Mat6Map = ::Eigen::Map<::Eigen::Matrix<double, 6, 6>>;

inline SE3 se3_from_hom(const double m[16]) {
    Mat4Map H{m};
    return SE3{H.topLeftCorner<3, 3>(), H.topRightCorner<3, 1>()};
}

}  // namespace

void se3_log6(const double m_hom[16], double out6[6]) {
    const SE3 M = se3_from_hom(m_hom);
    Vec6Map v{out6};
    v = ::pinocchio::log6(M).toVector();
}

void se3_jlog6(const double m_hom[16], double out36[36]) {
    const SE3 M = se3_from_hom(m_hom);
    Mat6Map J{out36};
    ::pinocchio::Jlog6(M, J);
}

namespace {

using VecXMap = ::Eigen::Map<const ::Eigen::VectorXd>;
using VecXMapMut = ::Eigen::Map<::Eigen::VectorXd>;
using MatXMapMut = ::Eigen::Map<::Eigen::MatrixXd>;
using HomMapMut = ::Eigen::Map<::Eigen::Matrix<double, 4, 4>>;
using V6MapMut = ::Eigen::Map<::Eigen::Matrix<double, 6, 1>>;

inline void write_hom(const SE3& M, double out16[16]) {
    HomMapMut H{out16};
    H = M.toHomogeneousMatrix();
}

inline ::pinocchio::ReferenceFrame to_pin_rf(std::uint8_t rf) {
    switch (rf) {
        case 0: return ::pinocchio::WORLD;
        case 1: return ::pinocchio::LOCAL;
        case 2: return ::pinocchio::LOCAL_WORLD_ALIGNED;
    }
    // Invalid value falls through to LOCAL — the Rust layer enforces validity.
    return ::pinocchio::LOCAL;
}

}  // namespace

void forward_kinematics(const Model& model, Data& data,
                        const double* q_ptr, std::size_t nq) {
    VecXMap q{q_ptr, static_cast<Eigen::Index>(nq)};
    ::pinocchio::forwardKinematics(model, data, q);
}

void forward_kinematics_v(const Model& model, Data& data,
                          const double* q_ptr, std::size_t nq,
                          const double* v_ptr, std::size_t nv) {
    VecXMap q{q_ptr, static_cast<Eigen::Index>(nq)};
    VecXMap v{v_ptr, static_cast<Eigen::Index>(nv)};
    ::pinocchio::forwardKinematics(model, data, q, v);
}

void forward_kinematics_v_a(const Model& model, Data& data,
                            const double* q_ptr, std::size_t nq,
                            const double* v_ptr, std::size_t nv,
                            const double* a_ptr, std::size_t nv_a) {
    VecXMap q{q_ptr, static_cast<Eigen::Index>(nq)};
    VecXMap v{v_ptr, static_cast<Eigen::Index>(nv)};
    VecXMap a{a_ptr, static_cast<Eigen::Index>(nv_a)};
    ::pinocchio::forwardKinematics(model, data, q, v, a);
}

void update_frame_placements(const Model& model, Data& data) {
    ::pinocchio::updateFramePlacements(model, data);
}

void data_joint_placement(const Data& data, std::size_t joint_id, double out16[16]) {
    write_hom(data.oMi[joint_id], out16);
}

void data_frame_placement(const Data& data, std::size_t frame_id, double out16[16]) {
    write_hom(data.oMf[frame_id], out16);
}

void data_joint_velocity(const Data& data, std::size_t joint_id, double out6[6]) {
    V6MapMut v{out6};
    v = data.v[joint_id].toVector();
}

void model_neutral_configuration(const Model& model, double* out_q, std::size_t nq) {
    VecXMapMut q{out_q, static_cast<Eigen::Index>(nq)};
    q = ::pinocchio::neutral(model);
}

// ------- RNEA -------
void rnea(const Model& model, Data& data,
          const double* q_ptr, std::size_t nq,
          const double* v_ptr, std::size_t nv,
          const double* a_ptr, std::size_t nv_a) {
    VecXMap q{q_ptr, static_cast<Eigen::Index>(nq)};
    VecXMap v{v_ptr, static_cast<Eigen::Index>(nv)};
    VecXMap a{a_ptr, static_cast<Eigen::Index>(nv_a)};
    ::pinocchio::rnea(model, data, q, v, a);
}

void data_tau(const Data& data, double* out_tau, std::size_t nv) {
    VecXMapMut t{out_tau, static_cast<Eigen::Index>(nv)};
    t = data.tau;
}

// ------- ABA -------
void aba(const Model& model, Data& data,
         const double* q_ptr, std::size_t nq,
         const double* v_ptr, std::size_t nv,
         const double* tau_ptr, std::size_t nv_tau) {
    VecXMap q{q_ptr, static_cast<Eigen::Index>(nq)};
    VecXMap v{v_ptr, static_cast<Eigen::Index>(nv)};
    VecXMap tau{tau_ptr, static_cast<Eigen::Index>(nv_tau)};
    ::pinocchio::aba(model, data, q, v, tau);
}

void data_ddq(const Data& data, double* out_ddq, std::size_t nv) {
    VecXMapMut d{out_ddq, static_cast<Eigen::Index>(nv)};
    d = data.ddq;
}

// ------- CRBA -------
void crba(const Model& model, Data& data,
          const double* q_ptr, std::size_t nq) {
    VecXMap q{q_ptr, static_cast<Eigen::Index>(nq)};
    ::pinocchio::crba(model, data, q);
}

void data_mass_matrix_symmetrize(Data& data) {
    data.M.triangularView<Eigen::StrictlyLower>() =
        data.M.transpose().triangularView<Eigen::StrictlyLower>();
}

void data_mass_matrix_copy(const Data& data, double* out, std::size_t nv) {
    MatXMapMut M{out, static_cast<Eigen::Index>(nv), static_cast<Eigen::Index>(nv)};
    M = data.M;
}

// ------- Jacobians -------
void compute_joint_jacobian(const Model& model, Data& data,
                            const double* q_ptr, std::size_t nq,
                            std::size_t joint_id, double out6xnv[],
                            std::size_t nv) {
    VecXMap q{q_ptr, static_cast<Eigen::Index>(nq)};
    ::pinocchio::Data::Matrix6x J(6, static_cast<Eigen::Index>(nv));
    J.setZero();
    ::pinocchio::computeJointJacobian(model, data, q, joint_id, J);
    MatXMapMut out{out6xnv, 6, static_cast<Eigen::Index>(nv)};
    out = J;
}

void compute_frame_jacobian(const Model& model, Data& data,
                            const double* q_ptr, std::size_t nq,
                            std::size_t frame_id, std::uint8_t rf,
                            double out6xnv[], std::size_t nv) {
    VecXMap q{q_ptr, static_cast<Eigen::Index>(nq)};
    ::pinocchio::Data::Matrix6x J(6, static_cast<Eigen::Index>(nv));
    J.setZero();
    ::pinocchio::computeFrameJacobian(model, data, q, frame_id, to_pin_rf(rf), J);
    MatXMapMut out{out6xnv, 6, static_cast<Eigen::Index>(nv)};
    out = J;
}

// ------- Lie operations -------
void model_integrate(const Model& model,
                     const double* q_ptr, std::size_t nq,
                     const double* v_ptr, std::size_t nv,
                     double* out_q, std::size_t nq_out) {
    VecXMap q{q_ptr, static_cast<Eigen::Index>(nq)};
    VecXMap v{v_ptr, static_cast<Eigen::Index>(nv)};
    VecXMapMut out{out_q, static_cast<Eigen::Index>(nq_out)};
    out = ::pinocchio::integrate(model, q, v);
}

void model_difference(const Model& model,
                      const double* q0_ptr, const double* q1_ptr, std::size_t nq,
                      double* out_v, std::size_t nv_out) {
    VecXMap q0{q0_ptr, static_cast<Eigen::Index>(nq)};
    VecXMap q1{q1_ptr, static_cast<Eigen::Index>(nq)};
    VecXMapMut out{out_v, static_cast<Eigen::Index>(nv_out)};
    out = ::pinocchio::difference(model, q0, q1);
}

void model_random_configuration(const Model& model, std::uint64_t seed,
                                double* out_q, std::size_t nq) {
    // randomConfiguration uses Eigen's PRNG which is seeded by std::srand.
    // glibc's srand has well-known quirks for tiny seeds (srand(0) often
    // collides with srand(1) etc.), so we mix the user seed through a
    // splittable-PRNG step before handing it to libc.
    std::uint64_t mixed = seed ^ 0x9E3779B97F4A7C15ull;
    mixed *= 0xBF58476D1CE4E5B9ull;
    mixed ^= mixed >> 30;
    std::srand(static_cast<unsigned int>(mixed));
    VecXMapMut out{out_q, static_cast<Eigen::Index>(nq)};
    out = ::pinocchio::randomConfiguration(model);
}

// ------- Derivatives -------
void compute_forward_kinematics_derivatives(const Model& model, Data& data,
                                            const double* q_ptr, std::size_t nq,
                                            const double* v_ptr, std::size_t nv,
                                            const double* a_ptr, std::size_t nv_a) {
    VecXMap q{q_ptr, static_cast<Eigen::Index>(nq)};
    VecXMap v{v_ptr, static_cast<Eigen::Index>(nv)};
    VecXMap a{a_ptr, static_cast<Eigen::Index>(nv_a)};
    ::pinocchio::computeForwardKinematicsDerivatives(model, data, q, v, a);
}

void data_joint_velocity_derivatives(const Model& model, Data& data,
                                     std::size_t joint_id, std::uint8_t rf,
                                     double* dv_dq, double* dv_dv,
                                     std::size_t nv) {
    ::pinocchio::Data::Matrix6x partial_dq(6, static_cast<Eigen::Index>(nv));
    ::pinocchio::Data::Matrix6x partial_dv(6, static_cast<Eigen::Index>(nv));
    partial_dq.setZero();
    partial_dv.setZero();
    ::pinocchio::getJointVelocityDerivatives(
        model, data, joint_id, to_pin_rf(rf), partial_dq, partial_dv);
    MatXMapMut out_dq{dv_dq, 6, static_cast<Eigen::Index>(nv)};
    MatXMapMut out_dv{dv_dv, 6, static_cast<Eigen::Index>(nv)};
    out_dq = partial_dq;
    out_dv = partial_dv;
}

}  // namespace pinocchio_rs::shim
