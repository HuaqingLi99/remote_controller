
#include "move_cartesian.h"
#include <limits>

#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <algorithm>
#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/rate_limiting.h>

/**
 * Single-target Cartesian impedance motion.
 *
 * This controller reads the current Franka O_T_EE pose, builds a synchronized
 * trapezoidal trajectory to one desired pose, and tracks it with Cartesian
 * impedance torques. Unlike moveC_path, this file uses Cartesian velocity
 * feed-forward from the generated trapezoid.
 */


namespace {

struct CartesianSample {
    double time;
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
    Eigen::Vector3d v_linear;
    Eigen::Vector3d v_angular;
};

struct TrapezoidProfile {
    double distance;
    double t_ramp;
    double t_total;
    double v_peak;
};

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

TrapezoidProfile makeTrapezoidProfile(double distance, double vmax, double acc_max) {
    TrapezoidProfile profile{};
    profile.distance = std::max(distance, 1e-9);
    profile.t_ramp = vmax / acc_max;
    const double d_ramp = 0.5 * acc_max * profile.t_ramp * profile.t_ramp;
    profile.v_peak = vmax;

    if (2.0 * d_ramp >= profile.distance) {
        profile.t_ramp = std::sqrt(profile.distance / acc_max);
        profile.v_peak = acc_max * profile.t_ramp;
        profile.t_total = 2.0 * profile.t_ramp;
    } else {
        const double d_const = profile.distance - 2.0 * d_ramp;
        const double t_const = d_const / vmax;
        profile.t_total = 2.0 * profile.t_ramp + t_const;
    }

    return profile;
}

void evaluateTrapezoid(
    const TrapezoidProfile& profile,
    double acc_max,
    double t,
    double& s,
    double& s_dot) {
    const double d_ramp = 0.5 * acc_max * profile.t_ramp * profile.t_ramp;

    if (t <= profile.t_ramp) {
        s = 0.5 * acc_max * t * t;
        s_dot = acc_max * t;
    } else if (t <= profile.t_total - profile.t_ramp) {
        s = d_ramp + profile.v_peak * (t - profile.t_ramp);
        s_dot = profile.v_peak;
    } else {
        const double t3 = t - (profile.t_total - profile.t_ramp);
        const double d_before = profile.distance - d_ramp;
        s = d_before + profile.v_peak * t3 - 0.5 * acc_max * t3 * t3;
        s_dot = std::max(profile.v_peak - acc_max * t3, 0.0);
    }

    s = std::max(0.0, std::min(s, profile.distance));
}


Eigen::Matrix4d arrayToMatrix4d(const std::array<double, 16>& T) {
    Eigen::Matrix4d M;
    for (int i = 0; i < 16; ++i) {
        // Python flatten_pose() sends row-major 4x4 values.
        // Keep this parser in row-major order to preserve translation entries.
        M(i / 4, i % 4) = T[i];
    }
    return M;
}

Eigen::Vector3d rotationError(
    const Eigen::Matrix3d& R,
    const Eigen::Matrix3d& R_d) {
    Eigen::Matrix3d R_err = R_d * R.transpose();
    Eigen::AngleAxisd aa(R_err);
    // Return axis * angle: a 3D rotation vector that points from current
    // orientation R toward desired orientation R_d.
    return aa.axis() * aa.angle();
}

std::vector<CartesianSample> make_cartesian_trajectory(
    const Eigen::Vector3d& p_start,
    const Eigen::Quaterniond& q_start,
    const Eigen::Vector3d& p_goal,
    const Eigen::Quaterniond& q_goal,
    double vmax_linear,
    double acc_max_linear,
    double vmax_angular,
    double acc_max_angular,
    double dt) {
    if (dt <= 0.0) {
        throw std::runtime_error("dt must be positive");
    }
    if (vmax_linear <= 0.0 || acc_max_linear <= 0.0 ||
        vmax_angular <= 0.0 || acc_max_angular <= 0.0) {
        throw std::runtime_error("linear and angular limits must be positive");
    }

    const Eigen::Vector3d position_delta = p_goal - p_start;
    const double position_distance = position_delta.norm();

    Eigen::Quaterniond q_goal_shortest = q_goal;
    if (q_start.dot(q_goal_shortest) < 0.0) {
        q_goal_shortest.coeffs() *= -1.0;
    }

    const Eigen::Vector3d rotation_vector_total =
        quaternionRotationVector(q_start, q_goal_shortest);
    const double rotation_angle = rotation_vector_total.norm();

    if (position_distance < 1e-9 && rotation_angle < 1e-9) {
        CartesianSample sample{};
        sample.time = 0.0;
        sample.p = p_goal;
        sample.q = q_goal_shortest;
        sample.v_linear.setZero();
        sample.v_angular.setZero();
        return {sample};
    }

    double alpha_dot_max = std::numeric_limits<double>::infinity();
    double alpha_ddot_max = std::numeric_limits<double>::infinity();

    if (position_distance >= 1e-9) {
        alpha_dot_max = std::min(alpha_dot_max, vmax_linear / position_distance);
        alpha_ddot_max = std::min(alpha_ddot_max, acc_max_linear / position_distance);
    }
    if (rotation_angle >= 1e-9) {
        alpha_dot_max = std::min(alpha_dot_max, vmax_angular / rotation_angle);
        alpha_ddot_max = std::min(alpha_ddot_max, acc_max_angular / rotation_angle);
    }

    if (!std::isfinite(alpha_dot_max) || !std::isfinite(alpha_ddot_max) ||
        alpha_dot_max <= 0.0 || alpha_ddot_max <= 0.0) {
        throw std::runtime_error("failed to compute valid pose timing limits");
    }

    const TrapezoidProfile profile = makeTrapezoidProfile(1.0, alpha_dot_max, alpha_ddot_max);

    int steps = static_cast<int>(std::ceil(profile.t_total / dt)) + 1;
    if (steps < 2) {
        steps = 2;
    }

    std::vector<CartesianSample> traj;
    traj.reserve(steps);

    for (int k = 0; k < steps; ++k) {
        const double t = std::min(k * dt, profile.t_total);

        double alpha = 0.0;
        double alpha_dot = 0.0;
        evaluateTrapezoid(profile, alpha_ddot_max, t, alpha, alpha_dot);

        CartesianSample sample{};
        sample.time = t;
        sample.p = p_start + alpha * position_delta;
        sample.q = q_start.slerp(alpha, q_goal_shortest).normalized();
        sample.v_linear = position_delta * alpha_dot;
        sample.v_angular = rotation_vector_total * alpha_dot;
        traj.push_back(sample);
    }

    traj.back().time = profile.t_total;
    traj.back().p = p_goal;
    traj.back().q = q_goal_shortest;
    traj.back().v_linear.setZero();
    traj.back().v_angular.setZero();

    return traj;
}
}

int moveCartesian(
    franka::Robot& robot,
    const std::array<double, 16>& T_d,
    const std::array<std::array<double, 6>, 6>& stiffness,
    double vmax_linear,
    double acc_max_linear,
    double vmax_angular,
    double acc_max_angular,
    double nullspace_stiffness,
    const std::function<void(const franka::RobotState&)>& state_callback,
    const std::function<double()>& slowdown_factor_callback,
    const std::function<bool()>& should_stop) {
    franka::Model model = robot.loadModel();
    franka::RobotState initial_state = robot.readOnce();

    Eigen::Matrix4d T_start = Eigen::Map<const Eigen::Matrix4d>(initial_state.O_T_EE.data());
    Eigen::Matrix4d T_goal = arrayToMatrix4d(T_d);

    Eigen::Vector3d p_start = T_start.block<3, 1>(0, 3);
    Eigen::Vector3d p_goal = T_goal.block<3, 1>(0, 3);

    Eigen::Quaterniond q_start(T_start.block<3, 3>(0, 0));
    Eigen::Quaterniond q_goal(T_goal.block<3, 3>(0, 0));
    q_start.normalize();
    q_goal.normalize();

    // Hold the initial joint posture in the nullspace unless the caller sets
    // nullspace_stiffness to zero.
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> q_start_joint(initial_state.q.data());

    std::vector<CartesianSample> trajectory =
        make_cartesian_trajectory(p_start, q_start, p_goal, q_goal, vmax_linear, acc_max_linear, vmax_angular, acc_max_angular, 0.001);

    double motion_time = 0.0;
    size_t sample_idx = 0;

    Eigen::Matrix<double, 6, 6> Kx = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 6> Dx = Eigen::Matrix<double, 6, 6>::Zero();

    double zeta = 1.0;
    for (size_t i = 0; i < 6; ++i) {
        for (size_t j = 0; j < 6; ++j) {
            Kx(i, j) = stiffness[i][j];
        }
        Dx(i, i) = 2.0 * zeta * std::sqrt(std::max(Kx(i, i), 0.0));
    }

    auto torque_callback =
        [&](const franka::RobotState& robot_state,
            franka::Duration duration) -> franka::Torques {
            const double dt = duration.toSec();

            double slowdown_factor = slowdown_factor_callback();
            if (!std::isfinite(slowdown_factor) || slowdown_factor < 1.0) {
                slowdown_factor = 1.0;
            }
            motion_time += dt / slowdown_factor;

            state_callback(robot_state);

            while (sample_idx + 1 < trajectory.size() &&
                   trajectory[sample_idx+1].time <= motion_time) {
                sample_idx++;
            }

            size_t current_idx = std::min(sample_idx, trajectory.size() - 1);
            const auto& sample = trajectory[current_idx];

            Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

            Eigen::Matrix4d T = Eigen::Map<const Eigen::Matrix4d>(robot_state.O_T_EE.data());
            Eigen::Vector3d p = T.block<3, 1>(0, 3);
            Eigen::Matrix3d R = T.block<3, 3>(0, 0);

            Eigen::Matrix3d R_d = sample.q.toRotationMatrix();

            Eigen::Vector3d p_error = sample.p - p;
            Eigen::Vector3d r_error = rotationError(R, R_d);

            Eigen::Matrix<double, 6, 1> error;
            error.head<3>() = p_error;
            error.tail<3>() = r_error;

            std::array<double, 42> jacobian_array =
                model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
            Eigen::Map<const Eigen::Matrix<double, 6, 7>> J(jacobian_array.data());

            Eigen::Matrix<double, 6, 1> dq_cart = J * dq;

            Eigen::Matrix<double, 6, 1> dxd_ref;
            dxd_ref.head<3>() = sample.v_linear / slowdown_factor;
            dxd_ref.tail<3>() = sample.v_angular / slowdown_factor;

            // Single moveCartesian has trapezoidal Cartesian feed-forward
            // velocity. moveC_path intentionally uses replayed poses only.
            Eigen::Matrix<double, 6, 1> derror = dxd_ref - dq_cart;

            std::array<double, 7> coriolis_array = model.coriolis(robot_state);
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
            
            if (should_stop()) {
                constexpr double kStopDamping = 20.0;

                // Stop by returning damped torques instead of abruptly dropping
                // the control callback.
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

            Eigen::Matrix<double, 7, 1> tau_task = J.transpose() * (Kx * error + Dx * derror);

            Eigen::Matrix<double, 7, 1> q_null_d = q_start_joint;
            Eigen::Matrix<double, 7, 1> tau_null =
                nullspace_stiffness * (q_null_d - q)
                - 2.0 * std::sqrt(std::max(nullspace_stiffness, 0.0)) * dq;

            // Damped Jacobian pseudoinverse
            const double lambda = 1e-3;
            Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
            Eigen::Matrix<double, 6, 6> damped =
                JJt + lambda * Eigen::Matrix<double, 6, 6>::Identity();
            Eigen::Matrix<double, 7, 6> J_pinv =
                J.transpose() * damped.inverse();

            // Nullspace projector
            Eigen::Matrix<double, 7, 7> N =
                Eigen::Matrix<double, 7, 7>::Identity() - J_pinv * J;

            // Final torque
            Eigen::Matrix<double, 7, 1> tau_d =
                tau_task + N * tau_null + coriolis;

            std::array<double, 7> tau_d_array{};
            Eigen::Map<Eigen::Matrix<double, 7, 1>>(tau_d_array.data()) = tau_d;
            tau_d_array = franka::limitRate(
                franka::kMaxTorqueRate,
                tau_d_array,
                robot_state.tau_J_d
            );



            if (motion_time >= trajectory.back().time) {
                std::cout << "moveCartesian finished at trajectory end." << std::endl;
                return franka::MotionFinished(franka::Torques(tau_d_array));
            }


            return franka::Torques(tau_d_array);
        };

    robot.control(torque_callback);
    std::cout << "Cartesian motion finished." << std::endl;
    return 0;
}
