// SPDX-License-Identifier: BSD-2-Clause
//
// Reference oracle binary. Links Pinocchio directly through the same pixi
// env that pinocchio-sys uses, samples deterministic inputs from
// std::mt19937(42), runs each algorithm, and emits JSON files matching
// SCHEMA.md.

#include <pinocchio/config.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/aba-derivatives.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/frames-derivatives.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace {

using Model = pinocchio::Model;
using Data = pinocchio::Data;

constexpr std::uint64_t SEED = 42;

// -- JSON helpers (no external dep) -----------------------------------------

class Json {
public:
    Json() {
        oss_ << std::setprecision(17);  // round-trip-stable doubles
    }

    void key(const char* k) {
        if (!first_in_obj_) {
            oss_ << ",";
        }
        first_in_obj_ = false;
        oss_ << "\"" << k << "\":";
    }

    void write_int(long long v) {
        oss_ << v;
    }

    void write_str(const std::string& s) {
        oss_ << "\"" << s << "\"";
    }

    // Flat array of doubles. We always emit even the last value without a
    // trailing comma so output is valid JSON.
    template <typename Derived>
    void write_eigen(const Eigen::MatrixBase<Derived>& mat) {
        oss_ << "[";
        const Eigen::Index n = mat.size();
        // Column-major flatten.
        const double* data = mat.derived().data();
        for (Eigen::Index i = 0; i < n; ++i) {
            if (i) oss_ << ",";
            oss_ << data[i];
        }
        oss_ << "]";
    }

    void begin_obj() {
        oss_ << "{";
        first_in_obj_ = true;
    }
    void end_obj() {
        oss_ << "}";
    }
    void begin_arr() {
        oss_ << "[";
        first_in_arr_.push_back(true);
    }
    void next_arr_elem() {
        if (!first_in_arr_.back()) {
            oss_ << ",";
        }
        first_in_arr_.back() = false;
    }
    void end_arr() {
        oss_ << "]";
        first_in_arr_.pop_back();
    }

    std::string str() const { return oss_.str(); }

private:
    std::ostringstream oss_;
    bool first_in_obj_ = true;
    std::vector<bool> first_in_arr_;
};

void write_file(const std::string& path, const std::string& content) {
    std::ofstream f(path);
    if (!f) {
        std::fprintf(stderr, "failed to open %s for writing\n", path.c_str());
        std::exit(1);
    }
    f << content;
}

// Sample a configuration vector q from the joint limits.
Eigen::VectorXd sample_q(const Model& model, std::mt19937& rng) {
    Eigen::VectorXd q = pinocchio::neutral(model);
    for (Eigen::Index i = 0; i < model.nq; ++i) {
        const double lo = model.lowerPositionLimit[i];
        const double hi = model.upperPositionLimit[i];
        if (std::isfinite(lo) && std::isfinite(hi) && hi > lo) {
            std::uniform_real_distribution<double> u{lo, hi};
            q[i] = u(rng);
        }
    }
    return q;
}

Eigen::VectorXd sample_small(Eigen::Index n, std::mt19937& rng) {
    std::uniform_real_distribution<double> u{-0.1, 0.1};
    Eigen::VectorXd v(n);
    for (Eigen::Index i = 0; i < n; ++i) v[i] = u(rng);
    return v;
}

// SE(3) sampler reusing the URDF-loaded model to produce realistic poses.
pinocchio::SE3 sample_se3(std::mt19937& rng) {
    std::uniform_real_distribution<double> ang{-M_PI, M_PI};
    std::uniform_real_distribution<double> pos{-1.0, 1.0};
    Eigen::Vector3d axis{ang(rng), ang(rng), ang(rng)};
    axis.normalize();
    const double theta = ang(rng);
    const Eigen::AngleAxisd aa(theta, axis);
    const Eigen::Vector3d t{pos(rng), pos(rng), pos(rng)};
    return pinocchio::SE3{aa.toRotationMatrix(), t};
}

void write_envelope_header(Json& j, std::size_t sample_count) {
    j.begin_obj();
    j.key("robot");
    j.write_str("panda");
    j.key("pinocchio_version");
    j.write_str(PINOCCHIO_VERSION);
    j.key("seed");
    j.write_int(static_cast<long long>(SEED));
    j.key("sample_count");
    j.write_int(static_cast<long long>(sample_count));
    j.key("samples");
    j.begin_arr();
}

void write_envelope_footer(Json& j) {
    j.end_arr();
    j.end_obj();
}

// -- per-algorithm emitters --------------------------------------------------

void emit_fk(const std::string& outdir, const Model& model) {
    Data data(model);
    std::mt19937 rng(SEED ^ 0xF1u);
    Json j;
    constexpr std::size_t N = 100;
    write_envelope_header(j, N);
    for (std::size_t s = 0; s < N; ++s) {
        const auto q = sample_q(model, rng);
        pinocchio::forwardKinematics(model, data, q);
        j.next_arr_elem();
        j.begin_obj();
        j.key("q");
        j.write_eigen(q);
        j.key("oMi");
        // njoints × 16, flattened (column-major per matrix, joints concatenated).
        Eigen::VectorXd flat(static_cast<Eigen::Index>(model.njoints) * 16);
        for (int i = 0; i < model.njoints; ++i) {
            Eigen::Matrix4d H = data.oMi[i].toHomogeneousMatrix();
            for (int k = 0; k < 16; ++k) {
                flat[i * 16 + k] = H.data()[k];
            }
        }
        j.write_eigen(flat);
        j.end_obj();
    }
    write_envelope_footer(j);
    write_file(outdir + "/fk_panda.json", j.str());
}

void emit_rnea(const std::string& outdir, const Model& model) {
    Data data(model);
    std::mt19937 rng(SEED ^ 0xF2u);
    Json j;
    constexpr std::size_t N = 100;
    write_envelope_header(j, N);
    for (std::size_t s = 0; s < N; ++s) {
        const auto q = sample_q(model, rng);
        const auto v = sample_small(model.nv, rng);
        const auto a = sample_small(model.nv, rng);
        pinocchio::rnea(model, data, q, v, a);
        j.next_arr_elem();
        j.begin_obj();
        j.key("q"); j.write_eigen(q);
        j.key("v"); j.write_eigen(v);
        j.key("a"); j.write_eigen(a);
        j.key("tau"); j.write_eigen(data.tau);
        j.end_obj();
    }
    write_envelope_footer(j);
    write_file(outdir + "/rnea_panda.json", j.str());
}

void emit_aba(const std::string& outdir, const Model& model) {
    Data data(model);
    std::mt19937 rng(SEED ^ 0xF3u);
    Json j;
    constexpr std::size_t N = 100;
    write_envelope_header(j, N);
    for (std::size_t s = 0; s < N; ++s) {
        const auto q = sample_q(model, rng);
        const auto v = sample_small(model.nv, rng);
        const auto tau = sample_small(model.nv, rng);
        pinocchio::aba(model, data, q, v, tau);
        j.next_arr_elem();
        j.begin_obj();
        j.key("q"); j.write_eigen(q);
        j.key("v"); j.write_eigen(v);
        j.key("tau"); j.write_eigen(tau);
        j.key("ddq"); j.write_eigen(data.ddq);
        j.end_obj();
    }
    write_envelope_footer(j);
    write_file(outdir + "/aba_panda.json", j.str());
}

void emit_crba(const std::string& outdir, const Model& model) {
    Data data(model);
    std::mt19937 rng(SEED ^ 0xF4u);
    Json j;
    constexpr std::size_t N = 100;
    write_envelope_header(j, N);
    for (std::size_t s = 0; s < N; ++s) {
        const auto q = sample_q(model, rng);
        pinocchio::crba(model, data, q);
        // Symmetrize (matches the safe wrapper).
        data.M.triangularView<Eigen::StrictlyLower>() =
            data.M.transpose().triangularView<Eigen::StrictlyLower>();
        j.next_arr_elem();
        j.begin_obj();
        j.key("q"); j.write_eigen(q);
        j.key("M"); j.write_eigen(data.M);
        j.end_obj();
    }
    write_envelope_footer(j);
    write_file(outdir + "/crba_panda.json", j.str());
}

void emit_jac(const std::string& outdir, const Model& model) {
    Data data(model);
    std::mt19937 rng(SEED ^ 0xF5u);
    Json j;
    constexpr std::size_t N = 50;
    write_envelope_header(j, N);
    // Use panda_hand frame for jacobian sampling — it's present and exercises FK.
    const auto frame_id = model.getFrameId("panda_hand");
    for (std::size_t s = 0; s < N; ++s) {
        const auto q = sample_q(model, rng);
        const int rf_int = static_cast<int>(s % 3);
        pinocchio::ReferenceFrame rf =
            (rf_int == 0) ? pinocchio::WORLD :
            (rf_int == 1) ? pinocchio::LOCAL : pinocchio::LOCAL_WORLD_ALIGNED;
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, model.nv);
        pinocchio::computeFrameJacobian(model, data, q, frame_id, rf, J);
        j.next_arr_elem();
        j.begin_obj();
        j.key("q"); j.write_eigen(q);
        j.key("frame_id"); j.write_int(static_cast<long long>(frame_id));
        j.key("rf"); j.write_int(static_cast<long long>(rf_int));
        j.key("J"); j.write_eigen(J);
        j.end_obj();
    }
    write_envelope_footer(j);
    write_file(outdir + "/jac_panda.json", j.str());
}

void emit_se3(const std::string& outdir) {
    std::mt19937 rng(SEED ^ 0xF6u);
    Json j;
    constexpr std::size_t N = 500;
    write_envelope_header(j, N);
    for (std::size_t s = 0; s < N; ++s) {
        const pinocchio::SE3 M = sample_se3(rng);
        Eigen::Matrix4d H = M.toHomogeneousMatrix();
        Eigen::Matrix4d Hinv = M.inverse().toHomogeneousMatrix();
        Eigen::Matrix<double, 6, 1> lv = pinocchio::log6(M).toVector();
        Eigen::Matrix<double, 6, 6> jl;
        pinocchio::Jlog6(M, jl);
        j.next_arr_elem();
        j.begin_obj();
        j.key("m_in"); j.write_eigen(H);
        j.key("inv"); j.write_eigen(Hinv);
        j.key("log6"); j.write_eigen(lv);
        j.key("jlog6"); j.write_eigen(jl);
        j.end_obj();
    }
    write_envelope_footer(j);
    write_file(outdir + "/se3_ops.json", j.str());
}

void emit_lie(const std::string& outdir, const Model& model) {
    std::mt19937 rng(SEED ^ 0xF7u);
    Json j;
    constexpr std::size_t N = 500;
    write_envelope_header(j, N);
    for (std::size_t s = 0; s < N; ++s) {
        const auto q0 = sample_q(model, rng);
        const auto q1 = sample_q(model, rng);
        const auto v = pinocchio::difference(model, q0, q1);
        const auto qb = pinocchio::integrate(model, q0, v);
        j.next_arr_elem();
        j.begin_obj();
        j.key("q0"); j.write_eigen(q0);
        j.key("q1"); j.write_eigen(q1);
        j.key("v"); j.write_eigen(v);
        j.key("q_back"); j.write_eigen(qb);
        j.end_obj();
    }
    write_envelope_footer(j);
    write_file(outdir + "/lie_ops.json", j.str());
}

void emit_derivs(const std::string& outdir, const Model& model) {
    Data data(model);
    std::mt19937 rng(SEED ^ 0xF8u);
    Json j;
    constexpr std::size_t N = 50;
    write_envelope_header(j, N);
    for (std::size_t s = 0; s < N; ++s) {
        const auto q = sample_q(model, rng);
        const auto v = sample_small(model.nv, rng);
        const auto a = sample_small(model.nv, rng);
        pinocchio::computeForwardKinematicsDerivatives(model, data, q, v, a);
        // Pick a non-trivial joint (joint 4) and the Local frame.
        const std::size_t joint_id = 4;
        Eigen::MatrixXd dq = Eigen::MatrixXd::Zero(6, model.nv);
        Eigen::MatrixXd dv = Eigen::MatrixXd::Zero(6, model.nv);
        pinocchio::getJointVelocityDerivatives(
            model, data, joint_id, pinocchio::LOCAL, dq, dv);
        j.next_arr_elem();
        j.begin_obj();
        j.key("q"); j.write_eigen(q);
        j.key("v"); j.write_eigen(v);
        j.key("a"); j.write_eigen(a);
        j.key("joint_id"); j.write_int(static_cast<long long>(joint_id));
        j.key("rf"); j.write_int(1);  // Local
        j.key("dv_dq"); j.write_eigen(dq);
        j.key("dv_dv"); j.write_eigen(dv);
        j.end_obj();
    }
    write_envelope_footer(j);
    write_file(outdir + "/fk_derivs_panda.json", j.str());
}

// ---- new derivative streams ----

void emit_rnea_derivs(const std::string& outdir, const Model& model) {
    Data data(model);
    std::mt19937 rng(SEED ^ 0xF9u);
    Json j;
    constexpr std::size_t N = 50;
    write_envelope_header(j, N);
    for (std::size_t s = 0; s < N; ++s) {
        const auto q = sample_q(model, rng);
        const auto v = sample_small(model.nv, rng);
        const auto a = sample_small(model.nv, rng);
        pinocchio::computeRNEADerivatives(model, data, q, v, a);
        // data.dtau_dq / data.dtau_dv are `RowMatrixXs` (row-major). Convert
        // to column-major to match the file's documented storage order.
        const Eigen::MatrixXd dtau_dq_col = data.dtau_dq;
        const Eigen::MatrixXd dtau_dv_col = data.dtau_dv;
        j.next_arr_elem();
        j.begin_obj();
        j.key("q"); j.write_eigen(q);
        j.key("v"); j.write_eigen(v);
        j.key("a"); j.write_eigen(a);
        j.key("dtau_dq"); j.write_eigen(dtau_dq_col);
        j.key("dtau_dv"); j.write_eigen(dtau_dv_col);
        j.end_obj();
    }
    write_envelope_footer(j);
    write_file(outdir + "/rnea_derivs_panda.json", j.str());
}

void emit_aba_derivs(const std::string& outdir, const Model& model) {
    Data data(model);
    std::mt19937 rng(SEED ^ 0xFAu);
    Json j;
    constexpr std::size_t N = 50;
    write_envelope_header(j, N);
    for (std::size_t s = 0; s < N; ++s) {
        const auto q = sample_q(model, rng);
        const auto v = sample_small(model.nv, rng);
        const auto tau = sample_small(model.nv, rng);
        pinocchio::computeABADerivatives(model, data, q, v, tau);
        // Same RowMatrixXs → MatrixXs conversion as in emit_rnea_derivs.
        const Eigen::MatrixXd ddq_dq_col = data.ddq_dq;
        const Eigen::MatrixXd ddq_dv_col = data.ddq_dv;
        j.next_arr_elem();
        j.begin_obj();
        j.key("q"); j.write_eigen(q);
        j.key("v"); j.write_eigen(v);
        j.key("tau"); j.write_eigen(tau);
        j.key("dddq_dq"); j.write_eigen(ddq_dq_col);
        j.key("dddq_dv"); j.write_eigen(ddq_dv_col);
        j.end_obj();
    }
    write_envelope_footer(j);
    write_file(outdir + "/aba_derivs_panda.json", j.str());
}

void emit_frame_kinematics_derivs(const std::string& outdir, const Model& model) {
    Data data(model);
    std::mt19937 rng(SEED ^ 0xFBu);
    Json j;
    constexpr std::size_t N = 50;
    write_envelope_header(j, N);

    // Cycle through a small set of frame ids (mid-chain, late-chain, tool).
    const std::vector<std::string> frame_names = {
        "panda_link3", "panda_link5", "panda_hand",
    };
    std::vector<std::size_t> frame_ids;
    frame_ids.reserve(frame_names.size());
    for (const auto& n : frame_names) {
        frame_ids.push_back(model.getFrameId(n));
    }

    for (std::size_t s = 0; s < N; ++s) {
        const auto q = sample_q(model, rng);
        const auto v = sample_small(model.nv, rng);
        const auto a = sample_small(model.nv, rng);
        pinocchio::computeForwardKinematicsDerivatives(model, data, q, v, a);

        const int rf_int = static_cast<int>(s % 3);
        const pinocchio::ReferenceFrame rf =
            (rf_int == 0) ? pinocchio::WORLD :
            (rf_int == 1) ? pinocchio::LOCAL : pinocchio::LOCAL_WORLD_ALIGNED;
        const std::size_t frame_id = frame_ids[s % frame_ids.size()];

        Eigen::MatrixXd dv_dq = Eigen::MatrixXd::Zero(6, model.nv);
        Eigen::MatrixXd dv_dv = Eigen::MatrixXd::Zero(6, model.nv);
        pinocchio::getFrameVelocityDerivatives(
            model, data, frame_id, rf, dv_dq, dv_dv);

        Eigen::MatrixXd v_partial_dq = Eigen::MatrixXd::Zero(6, model.nv);
        Eigen::MatrixXd da_dq = Eigen::MatrixXd::Zero(6, model.nv);
        Eigen::MatrixXd da_dv = Eigen::MatrixXd::Zero(6, model.nv);
        Eigen::MatrixXd da_da = Eigen::MatrixXd::Zero(6, model.nv);
        pinocchio::getFrameAccelerationDerivatives(
            model, data, frame_id, rf,
            v_partial_dq, da_dq, da_dv, da_da);

        j.next_arr_elem();
        j.begin_obj();
        j.key("q"); j.write_eigen(q);
        j.key("v"); j.write_eigen(v);
        j.key("a"); j.write_eigen(a);
        j.key("frame_id"); j.write_int(static_cast<long long>(frame_id));
        j.key("rf"); j.write_int(rf_int);
        j.key("dv_dq"); j.write_eigen(dv_dq);
        j.key("dv_dv"); j.write_eigen(dv_dv);
        j.key("da_dq"); j.write_eigen(da_dq);
        j.key("da_dv"); j.write_eigen(da_dv);
        j.key("da_da"); j.write_eigen(da_da);
        j.end_obj();
    }
    write_envelope_footer(j);
    write_file(outdir + "/frame_kinematics_derivs_panda.json", j.str());
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 3) {
        std::fprintf(stderr,
                     "usage: gen_goldens <urdf-path> <output-dir>\n"
                     "  e.g. gen_goldens pinocchio-rs/tests/data/panda/panda.urdf\n"
                     "                   pinocchio-rs/tests/goldens\n");
        return 2;
    }
    const std::string urdf = argv[1];
    const std::string outdir = argv[2];

    Model model;
    pinocchio::urdf::buildModel(urdf, model);

    std::printf("gen_goldens: model loaded, nq=%d nv=%d (Pinocchio %s)\n",
                model.nq, model.nv, PINOCCHIO_VERSION);

    emit_fk(outdir, model);
    emit_rnea(outdir, model);
    emit_aba(outdir, model);
    emit_crba(outdir, model);
    emit_jac(outdir, model);
    emit_se3(outdir);
    emit_lie(outdir, model);
    emit_derivs(outdir, model);
    emit_rnea_derivs(outdir, model);
    emit_aba_derivs(outdir, model);
    emit_frame_kinematics_derivs(outdir, model);

    std::printf("gen_goldens: wrote 11 JSON files under %s/\n", outdir.c_str());
    return 0;
}
