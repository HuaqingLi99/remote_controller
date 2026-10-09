#include "move_cartesian_path.h"

#include <array>
#include <cmath>
#include <functional>
#include <algorithm>
#include <limits>
#include <stdexcept>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
/**
 * Multi-waypoint Cartesian impedance path replay.
 *
 * The controller prepends the current Franka O_T_EE pose, connects the provided
 * row-major 4x4 waypoint poses with linear or Catmull-Rom interpolation, then
 * replays the resulting pose sequence from the 1 kHz base period.
 * samples_per_segment controls path density, and the runtime motion slowdown
 * factor controls replay timing; acc_max_* is accepted by the RPC API but is
 * not used here.
 */
namespace {

constexpr double kDefaultDt = 0.001;      // 1 kHz replay
constexpr double kMinSegmentNorm = 1e-12;
constexpr double kRotationWeight = 0.2;   // [m/rad] for pose arc-length mixing
constexpr double kCentripetalAlpha = 0.5;

struct CartesianPose {
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
};

struct CartesianSample {
    double t;
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
};

using CartesianPath = std::vector<CartesianPose>;
using TimedCartesianPath = std::vector<CartesianSample>;

Eigen::Matrix4d arrayToMatrix4d(const std::array<double, 16>& T) {
    Eigen::Matrix4d M;
    for (int i = 0; i < 16; ++i) {
        // Python flatten_pose() sends row-major 4x4 values.
        // Do not replace this with a raw Eigen::Map unless the client changes.
        M(i / 4, i % 4) = T[i];
    }
    return M;
}

CartesianPose matrixToPose(const Eigen::Matrix4d& T) {
    CartesianPose pose;
    pose.p = T.block<3, 1>(0, 3);
    pose.q = Eigen::Quaterniond(T.block<3, 3>(0, 0));
    pose.q.normalize();
    return pose;
}

CartesianPose arrayToPose(const std::array<double, 16>& T) {
    return matrixToPose(arrayToMatrix4d(T));
}

Eigen::Vector3d quaternionRotationVector(
    const Eigen::Quaterniond& q_start,
    const Eigen::Quaterniond& q_goal) {
    Eigen::Quaterniond q_rel = q_start.conjugate() * q_goal;
    if (q_rel.w() < 0.0) {
        q_rel.coeffs() *= -1.0;
    }
    q_rel.normalize();

    Eigen::AngleAxisd aa(q_rel);
    if (std::abs(aa.angle()) < 1e-12) {
        return Eigen::Vector3d::Zero();
    }
    return aa.axis() * aa.angle();
}
double tjPosition(
    double ti,
    const Eigen::Vector3d& pi,
    const Eigen::Vector3d& pj,
    double alpha) {
    const double dist = (pj - pi).norm();
    return ti + std::pow(std::max(dist, kMinSegmentNorm), alpha);
}

Eigen::Vector3d catmullRomPosition(
    const Eigen::Vector3d& p0,
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& p2,
    const Eigen::Vector3d& p3,
    double u,
    double alpha) {

    // Centripetal Catmull-Rom parameterization reduces overshoot around
    // unevenly spaced Cartesian waypoints.
    const double t0 = 0.0;
    const double t1 = tjPosition(t0, p0, p1, alpha);
    const double t2 = tjPosition(t1, p1, p2, alpha);
    const double t3 = tjPosition(t2, p2, p3, alpha);

    const double t = t1 + u * (t2 - t1);

    Eigen::Vector3d A1 =
        ((t1 - t) / (t1 - t0)) * p0 +
        ((t - t0) / (t1 - t0)) * p1;

    Eigen::Vector3d A2 =
        ((t2 - t) / (t2 - t1)) * p1 +
        ((t - t1) / (t2 - t1)) * p2;

    Eigen::Vector3d A3 =
        ((t3 - t) / (t3 - t2)) * p2 +
        ((t - t2) / (t3 - t2)) * p3;

    Eigen::Vector3d B1 =
        ((t2 - t) / (t2 - t0)) * A1 +
        ((t - t0) / (t2 - t0)) * A2;

    Eigen::Vector3d B2 =
        ((t3 - t) / (t3 - t1)) * A2 +
        ((t - t1) / (t3 - t1)) * A3;

    Eigen::Vector3d C =
        ((t2 - t) / (t2 - t1)) * B1 +
        ((t - t1) / (t2 - t1)) * B2;

    return C;
}


double poseDistance(
    const CartesianPose& a,
    const CartesianPose& b,
    double rotation_weight) {
    const double dp = (b.p - a.p).norm();

    Eigen::Quaterniond q_b = b.q;
    if (a.q.dot(q_b) < 0.0) {
        q_b.coeffs() *= -1.0;
    }
    const double dtheta = quaternionRotationVector(a.q, q_b).norm();

    return dp + rotation_weight * dtheta;
}

CartesianPose interpolatePose(
    const CartesianPose& a,
    const CartesianPose& b,
    double alpha) {
    CartesianPose out;
    out.p = a.p + alpha * (b.p - a.p);

    Eigen::Quaterniond q_b = b.q;
    if (a.q.dot(q_b) < 0.0) {
        q_b.coeffs() *= -1.0;
    }
    out.q = a.q.slerp(alpha, q_b).normalized();
    return out;
}

CartesianPath arrayToCartesianPath(
    const Eigen::Matrix4d& start_pose,
    const std::vector<std::array<double, 16>>& waypoints) {
    CartesianPath path;
    path.reserve(waypoints.size() + 1);

    path.push_back(matrixToPose(start_pose));
    for (const auto& wp : waypoints) {
        path.push_back(arrayToPose(wp));
    }
    return path;
}

CartesianPath connectLinearCartesianPath(
    const CartesianPath& waypoints,
    int samples_per_segment) {
    if (waypoints.size() < 2) {
        throw std::runtime_error("At least 2 Cartesian waypoints are required.");
    }
    if (samples_per_segment <= 0) {
        throw std::runtime_error("samples_per_segment must be positive.");
    }

    CartesianPath result;
    result.reserve((waypoints.size() - 1) * static_cast<size_t>(samples_per_segment) + 1);

    for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
        const auto& start = waypoints[i];
        const auto& end = waypoints[i + 1];

        for (int j = 0; j < samples_per_segment; ++j) {
            const double u =
                static_cast<double>(j) / static_cast<double>(samples_per_segment);
            result.push_back(interpolatePose(start, end, u));
        }
    }

    result.push_back(waypoints.back());
    return result;
}




CartesianPath resampleByCartesianDistance(
    const CartesianPath& dense_path,
    int output_samples,
    double rotation_weight) {
    if (dense_path.size() < 2) {
        throw std::runtime_error("dense_path must have at least 2 points.");
    }
    if (output_samples < 2) {
        throw std::runtime_error("output_samples must be >= 2.");
    }

    double total_length = 0.0;
    for (size_t i = 1; i < dense_path.size(); ++i) {
        total_length += poseDistance(dense_path[i - 1], dense_path[i], rotation_weight);
    }

    if (total_length <= 1e-12) {
        throw std::runtime_error("Cartesian path length is too small.");
    }

    const double target_ds =
        total_length / static_cast<double>(output_samples - 1);

    CartesianPath result;
    result.reserve(output_samples);
    result.push_back(dense_path.front());

    double accumulated_s = 0.0;
    double next_target_s = target_ds;

    for (size_t i = 1; i < dense_path.size(); ++i) {
        CartesianPose seg_start = dense_path[i - 1];
        const CartesianPose& seg_end = dense_path[i];

        double segment_len = poseDistance(seg_start, seg_end, rotation_weight);
        if (segment_len <= 1e-12) {
            continue;
        }

        while (accumulated_s + segment_len >= next_target_s &&
               result.size() + 1 < static_cast<size_t>(output_samples)) {
            const double local_alpha =
                (next_target_s - accumulated_s) / segment_len;

            result.push_back(interpolatePose(seg_start, seg_end, local_alpha));
            next_target_s += target_ds;
        }

        accumulated_s += segment_len;
    }

    result.push_back(dense_path.back());
    return result;
}

CartesianPath connectCatmullRomCartesianPath(
    const CartesianPath& waypoints,
    int samples_per_segment,
    double alpha) {

    if (waypoints.size() < 2) {
        throw std::runtime_error("At least 2 Cartesian waypoints are required.");
    }
    if (samples_per_segment <= 0) {
        throw std::runtime_error("samples_per_segment must be positive.");
    }

    if (waypoints.size() <= 3) {
        return connectLinearCartesianPath(waypoints, samples_per_segment);
    }

    CartesianPath padded;
    padded.reserve(waypoints.size() + 2);
    padded.push_back(waypoints.front());

    for (const auto& p : waypoints) {
        padded.push_back(p);
    }

    padded.push_back(waypoints.back());

    constexpr int dense_multiplier = 10;
    const int dense_samples_per_segment =
        samples_per_segment * dense_multiplier;

    CartesianPath dense_path;
    dense_path.reserve(
        (waypoints.size() - 1) *
        static_cast<size_t>(dense_samples_per_segment) + 1
    );

    for (size_t seg = 0; seg + 3 < padded.size(); ++seg) {
        const CartesianPose& p0 = padded[seg];
        const CartesianPose& p1 = padded[seg + 1];
        const CartesianPose& p2 = padded[seg + 2];
        const CartesianPose& p3 = padded[seg + 3];

        for (int j = 0; j < dense_samples_per_segment; ++j) {
            const double u =
                static_cast<double>(j) /
                static_cast<double>(dense_samples_per_segment);

            CartesianPose pose;
            pose.p = catmullRomPosition(
                p0.p,
                p1.p,
                p2.p,
                p3.p,
                u,
                alpha
            );

            Eigen::Quaterniond q2 = p2.q;
            if (p1.q.dot(q2) < 0.0) {
                q2.coeffs() *= -1.0;
            }

            // Position uses Catmull-Rom, orientation follows the same segment
            // parameter u with shortest-path slerp from p1.q to p2.q.
            pose.q = p1.q.slerp(u, q2).normalized();

            dense_path.push_back(pose);
        }
    }
    dense_path.push_back(waypoints.back());
    return dense_path;


}

CartesianPath connectCartesianPath(
    const CartesianPath& waypoints,
    Pathmode mode,
    int samples_per_segment) {
    switch (mode) {
        case Pathmode::Linear:
            return connectLinearCartesianPath(waypoints, samples_per_segment);

        case Pathmode::CatmullRomSpline:
            return connectCatmullRomCartesianPath(
                waypoints,
                samples_per_segment,
                kCentripetalAlpha
            );

        default:
            throw std::runtime_error("Unsupported Cartesian path mode.");
    }
}



TimedCartesianPath buildTimedCartesianTrajectory(
    const CartesianPath& path,
    double dt) {

    if (path.empty()) {
        throw std::runtime_error("Cartesian path is empty.");
    }
    if (dt <= 0.0) {
        throw std::runtime_error("dt must be positive.");
    }

    TimedCartesianPath traj;
    traj.reserve(path.size());

    for (size_t k = 0; k < path.size(); ++k) {
        CartesianSample sample{};
        // Path replay timing is sample-index based. Velocity limits are checked
        // before execution; there is no separate acceleration profile here.
        sample.t = static_cast<double>(k) * dt;
        sample.p = path[k].p;
        sample.q = path[k].q;
        traj.push_back(sample);
    }

    return traj;
}


struct ReplayCheckResult {
    bool ok{true};
    size_t bad_segment_idx{0};
    double actual_linear_velocity{0.0};
    double linear_velocity_limit{0.0};
    double actual_angular_velocity{0.0};
    double angular_velocity_limit{0.0};
    std::string message;
};

ReplayCheckResult checkCartesianReplayFeasibility(
    const CartesianPath& path,
    double dt,
    double vmax_linear,
    double vmax_angular) {
    ReplayCheckResult result;

    if (path.size() < 2) {
        result.ok = false;
        result.message = "Cartesian path must contain at least 2 points.";
        return result;
    }
    if (dt <= 0.0) {
        result.ok = false;
        result.message = "dt must be positive.";
        return result;
    }
    if (vmax_linear <= 0.0 || vmax_angular <= 0.0) {
        result.ok = false;
        result.message = "vmax_linear and vmax_angular must be positive.";
        return result;
    }

    for (size_t seg = 0; seg + 1 < path.size(); ++seg) {
        const double v_lin =
            (path[seg + 1].p - path[seg].p).norm() / dt;

        Eigen::Quaterniond q_next = path[seg + 1].q;
        if (path[seg].q.dot(q_next) < 0.0) {
            q_next.coeffs() *= -1.0;
        }

        const double v_ang =
            quaternionRotationVector(path[seg].q, q_next).norm() / dt;

        if (v_lin > vmax_linear + 1e-12) {
            result.ok = false;
            result.bad_segment_idx = seg;
            result.actual_linear_velocity = v_lin;
            result.linear_velocity_limit = vmax_linear;

            std::ostringstream oss;
            oss << "Cartesian replay rejected before execution: linear velocity limit exceeded. "
                << "Segment " << seg << " -> " << (seg + 1)
                << ", implied linear velocity = " << v_lin << " m/s"
                << ", vmax_linear = " << vmax_linear << " m/s. "
                << "Increase samples_per_segment or reduce waypoint spacing.";
            result.message = oss.str();
            return result;
        }

        if (v_ang > vmax_angular + 1e-12) {
            result.ok = false;
            result.bad_segment_idx = seg;
            result.actual_angular_velocity = v_ang;
            result.angular_velocity_limit = vmax_angular;

            std::ostringstream oss;
            oss << "Cartesian replay rejected before execution: angular velocity limit exceeded. "
                << "Segment " << seg << " -> " << (seg + 1)
                << ", implied angular velocity = " << v_ang << " rad/s"
                << ", vmax_angular = " << vmax_angular << " rad/s. "
                << "Increase samples_per_segment or reduce orientation spacing.";
            result.message = oss.str();
            return result;
        }
    }

    return result;
}

Eigen::Vector3d rotationError(
    const Eigen::Matrix3d& R,
    const Eigen::Matrix3d& R_d) {
    Eigen::Matrix3d R_err = R_d * R.transpose();
    Eigen::AngleAxisd aa(R_err);
    if (std::abs(aa.angle()) < 1e-12) {
        return Eigen::Vector3d::Zero();
    }
    return aa.axis() * aa.angle();
}

Eigen::Matrix<double, 6, 6> buildCartesianStiffnessMatrix(
    const std::array<std::array<double, 6>, 6>& stiffness) {
    Eigen::Matrix<double, 6, 6> Kx = Eigen::Matrix<double, 6, 6>::Zero();
    for (size_t i = 0; i < 6; ++i) {
        for (size_t j = 0; j < 6; ++j) {
            Kx(i, j) = stiffness[i][j];
        }
    }
    return Kx;
}

Eigen::Matrix<double, 6, 6> buildCartesianDampingMatrixFromStiffness(
    const std::array<std::array<double, 6>, 6>& stiffness,
    double zeta = 1.0) {
    Eigen::Matrix<double, 6, 6> Dx = Eigen::Matrix<double, 6, 6>::Zero();
    for (size_t i = 0; i < 6; ++i) {
        const double kii = std::max(stiffness[i][i], 0.0);
        Dx(i, i) = 2.0 * zeta * std::sqrt(kii);
    }
    return Dx;
}

}  // namespace

int moveC_path(
    franka::Robot& robot,
    const std::vector<std::array<double, 16>>& waypoints,
    int samples_per_segment,
    Pathmode path_mode,
    const std::array<std::array<double, 6>, 6>& stiffness,
    double vmax_linear,
    double acc_max_linear,
    double vmax_angular,
    double acc_max_angular,
    double nullspace_stiffness,
    const std::function<void(const franka::RobotState&)>& state_callback,
    const std::function<double()>& slowdown_factor_callback,
    const std::function<bool()>& should_stop) {

    (void)acc_max_linear;
    (void)acc_max_angular;

    if (waypoints.empty()) {
        throw std::runtime_error("waypoints is empty.");
    }
    if (samples_per_segment <= 0) {
        throw std::runtime_error("samples_per_segment must be positive.");
    }
    if (vmax_linear <= 0.0 || vmax_angular <= 0.0) {
        throw std::runtime_error("vmax_linear and vmax_angular must be positive.");
    }

    franka::Model model = robot.loadModel();
    franka::RobotState initial_state = robot.readOnce();

    const Eigen::Matrix4d T_start =
        Eigen::Map<const Eigen::Matrix4d>(initial_state.O_T_EE.data());

    Eigen::Map<const Eigen::Matrix<double, 7, 1>> q_start_joint(initial_state.q.data());

    CartesianPath raw_path = arrayToCartesianPath(T_start, waypoints);
    CartesianPath connected_path = connectCartesianPath(raw_path, path_mode, samples_per_segment);

    const int output_samples =
        static_cast<int>(raw_path.size() - 1) * samples_per_segment + 1;

    CartesianPath resampled_path =
        resampleByCartesianDistance(connected_path, output_samples, kRotationWeight);

    TimedCartesianPath trajectory =
        buildTimedCartesianTrajectory(resampled_path, kDefaultDt);

    if (trajectory.size() < 2) {
        throw std::runtime_error("Generated Cartesian trajectory is too short.");
    }

    {
        ReplayCheckResult precheck =
            checkCartesianReplayFeasibility(
                resampled_path,
                kDefaultDt,
                vmax_linear,
                vmax_angular
            );
        if (!precheck.ok) {
            throw std::runtime_error(precheck.message);
        }
    }

    const Eigen::Matrix<double, 6, 6> Kx =
        buildCartesianStiffnessMatrix(stiffness);
    const Eigen::Matrix<double, 6, 6> Dx =
        buildCartesianDampingMatrixFromStiffness(stiffness, 1.0);

    double replay_time = 0.0;
    size_t sample_idx = 0;

    auto torque_callback =
        [&](const franka::RobotState& robot_state,
            franka::Duration duration) -> franka::Torques {

        const double dt = duration.toSec();

        double slowdown_factor = slowdown_factor_callback();
        if (!std::isfinite(slowdown_factor) || slowdown_factor < 1.0) {
            slowdown_factor = 1.0;
        }

        replay_time += dt / slowdown_factor;

        while (sample_idx + 1 < trajectory.size() &&
               trajectory[sample_idx + 1].t <= replay_time) {
            ++sample_idx;
        }

        if (sample_idx >= trajectory.size()) {
            sample_idx = trajectory.size() - 1;
        }

        state_callback(robot_state);

        const CartesianSample& ref = trajectory[sample_idx];

        Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

        Eigen::Matrix4d T =
            Eigen::Map<const Eigen::Matrix4d>(robot_state.O_T_EE.data());

        Eigen::Vector3d p = T.block<3, 1>(0, 3);
        Eigen::Matrix3d R = T.block<3, 3>(0, 0);
        Eigen::Matrix3d R_d = ref.q.toRotationMatrix();

        const Eigen::Vector3d p_error = ref.p - p;
        const Eigen::Vector3d r_error = rotationError(R, R_d);

        Eigen::Matrix<double, 6, 1> error;
        error.head<3>() = p_error;
        error.tail<3>() = r_error;

        std::array<double, 42> jacobian_array =
            model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
        Eigen::Map<const Eigen::Matrix<double, 6, 7>> J(jacobian_array.data());

        const Eigen::Matrix<double, 6, 1> dx = J * dq;
        const Eigen::Matrix<double, 6, 1> derror = -dx;

        std::array<double, 7> coriolis_array = model.coriolis(robot_state);
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());

        if (should_stop()) {
            constexpr double kStopDamping = 20.0;

            std::array<double, 7> safe_tau{};
            for (size_t i = 0; i < 7; ++i) {
                safe_tau[i] = coriolis_array[i] - kStopDamping * robot_state.dq[i];
            }

            safe_tau = franka::limitRate(
                franka::kMaxTorqueRate,
                safe_tau,
                robot_state.tau_J_d
            );

            return franka::MotionFinished(franka::Torques(safe_tau));
        }

        Eigen::Matrix<double, 7, 1> tau_task =
            J.transpose() * (Kx * error + Dx * derror);

        Eigen::Matrix<double, 7, 1> q_null_d = q_start_joint;
        Eigen::Matrix<double, 7, 1> tau_null =
            nullspace_stiffness * (q_null_d - q)
            - 2.0 * std::sqrt(std::max(nullspace_stiffness, 0.0)) * dq;

        const double lambda = 1e-3;
        Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
        Eigen::Matrix<double, 6, 6> damped =
            JJt + lambda * lambda * Eigen::Matrix<double, 6, 6>::Identity();

        Eigen::Matrix<double, 7, 6> J_pinv =
            J.transpose() * damped.ldlt().solve(
                Eigen::Matrix<double, 6, 6>::Identity()
            );

        Eigen::Matrix<double, 7, 7> N =
            Eigen::Matrix<double, 7, 7>::Identity() - J_pinv * J;

        Eigen::Matrix<double, 7, 1> tau_d =
            tau_task + N * tau_null + coriolis;

        std::array<double, 7> tau_d_array{};
        Eigen::Map<Eigen::Matrix<double, 7, 1>>(tau_d_array.data()) = tau_d;

        tau_d_array = franka::limitRate(
            franka::kMaxTorqueRate,
            tau_d_array,
            robot_state.tau_J_d
        );

        const double traj_T = trajectory.back().t;

        if (replay_time >= traj_T) {
            return franka::MotionFinished(franka::Torques(tau_d_array));
        }

        return franka::Torques(tau_d_array);
    };

    robot.control(torque_callback);

    std::cout << "Cartesian path motion finished." << std::endl;
    return 0;
}