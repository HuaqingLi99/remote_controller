#include "moveJ_path.h"

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

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

/**
 * Multi-waypoint joint-space path replay.
 *
 * The current joint position is prepended to the waypoint list, the path is
 * connected with linear or Catmull-Rom interpolation, resampled by joint-space
 * distance, and replayed at 1 kHz. dq_max is checked before execution; ddq_max
 * is kept in the RPC/task shape for compatibility but is not used by replay.
 */

using JointVec = std::array<double, 7>;
using Path = std::vector<JointVec>;

struct TimedJointPoint {
    double t;
    JointVec q;
};

using TimedPath = std::vector<TimedJointPoint>;

namespace {

constexpr double kDefaultDt = 0.001;          // 1 kHz replay
constexpr double kMinSegmentNorm = 1e-12;
constexpr double kCentripetalAlpha = 0.5;     // default Catmull-Rom parameterization



double jointDistance(const JointVec& a, const JointVec& b) {
    double sum = 0.0;
    for (size_t i = 0; i < 7; ++i) {
        const double d = b[i] - a[i];
        sum += d * d;
    }
    return std::sqrt(sum);
}

JointVec interpolateJointVec(const JointVec& a, const JointVec& b, double alpha) {
    JointVec out{};
    for (size_t i = 0; i < 7; ++i) {
        out[i] = a[i] + alpha * (b[i] - a[i]);
    }
    return out;
}

Path arrayToPath(
    const std::array<double, 7>& startpoint,
    const std::vector<std::array<double, 7>>& waypoints) {
    Path path;
    path.reserve(waypoints.size() + 1);
    path.push_back(startpoint);
    for (const auto& wp : waypoints) {
        path.push_back(wp);
    }
    return path;
}

Path connectLinear(const Path& waypoints, int samples_per_segment) {
    if (waypoints.size() < 2) {
        throw std::runtime_error("At least 2 waypoints are required for linear path.");
    }
    if (samples_per_segment <= 0) {
        throw std::runtime_error("samples_per_segment must be positive.");
    }

    Path result;
    result.reserve((waypoints.size() - 1) * static_cast<size_t>(samples_per_segment) + 1);

    for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
        const auto& start = waypoints[i];
        const auto& end = waypoints[i + 1];

        for (int j = 0; j < samples_per_segment; ++j) {
            const double u = static_cast<double>(j) / static_cast<double>(samples_per_segment);
            result.push_back(interpolateJointVec(start, end, u));
        }
    }

    result.push_back(waypoints.back());
    return result;
}

double tj(double ti, const JointVec& pi, const JointVec& pj, double alpha) {
    const double dist = jointDistance(pi, pj);
    return ti + std::pow(std::max(dist, kMinSegmentNorm), alpha);
}

JointVec catmullRomPointGeneral(
    const JointVec& p0,
    const JointVec& p1,
    const JointVec& p2,
    const JointVec& p3,
    double u,
    double alpha) {

    const double t0 = 0.0;
    const double t1 = tj(t0, p0, p1, alpha);
    const double t2 = tj(t1, p1, p2, alpha);
    const double t3 = tj(t2, p2, p3, alpha);

    const double t = t1 + u * (t2 - t1);

    JointVec A1{}, A2{}, A3{};
    JointVec B1{}, B2{};
    JointVec C{};

    for (size_t i = 0; i < 7; ++i) {
        A1[i] = ((t1 - t) / (t1 - t0)) * p0[i] + ((t - t0) / (t1 - t0)) * p1[i];
        A2[i] = ((t2 - t) / (t2 - t1)) * p1[i] + ((t - t1) / (t2 - t1)) * p2[i];
        A3[i] = ((t3 - t) / (t3 - t2)) * p2[i] + ((t - t2) / (t3 - t2)) * p3[i];

        B1[i] = ((t2 - t) / (t2 - t0)) * A1[i] + ((t - t0) / (t2 - t0)) * A2[i];
        B2[i] = ((t3 - t) / (t3 - t1)) * A2[i] + ((t - t1) / (t3 - t1)) * A3[i];

        C[i] = ((t2 - t) / (t2 - t1)) * B1[i] + ((t - t1) / (t2 - t1)) * B2[i];
    }

    return C;
}
Path resampleByJointDistance(const Path& dense_path, int output_samples) {
    if (dense_path.size() < 2) {
        throw std::runtime_error("dense_path must have at least 2 points.");
    }
    if (output_samples < 2) {
        throw std::runtime_error("output_samples must be >= 2.");
    }

    double total_length = 0.0;
    for (size_t i = 1; i < dense_path.size(); ++i) {
        total_length += jointDistance(dense_path[i - 1], dense_path[i]);
    }//get total joint distance

    if (total_length <= 1e-12) {
        throw std::runtime_error("path length is too small.");
    }

    const double target_ds = total_length / static_cast<double>(output_samples - 1);//set the q_norm between sample point as nealy the same

    Path result;
    result.reserve(output_samples);
    result.push_back(dense_path.front());

    double accumulated_s = 0.0;
    double next_target_s = target_ds;

    for (size_t i = 1; i < dense_path.size(); ++i) {
        JointVec seg_start = dense_path[i - 1];
        const JointVec& seg_end = dense_path[i];

        double segment_len = jointDistance(seg_start, seg_end);
        if (segment_len <= 1e-12) {
            continue;
        }

        while (accumulated_s + segment_len >= next_target_s &&
               result.size() + 1 < static_cast<size_t>(output_samples)) {
            const double local_alpha =
                (next_target_s - accumulated_s) / segment_len;

            JointVec new_point = interpolateJointVec(
                seg_start,
                seg_end,
                local_alpha
            );

            result.push_back(new_point);
            next_target_s += target_ds;
        }

        accumulated_s += segment_len;
    }//find the of the sample and do interpolation to get the sample point

    result.push_back(dense_path.back());
    return result;
}

//the function we use to get dense points for the catmull-rom spline, with general parameterization
Path connectCatmullRomSpline(
    const Path& waypoints,
    int samples_per_segment,
    double alpha = kCentripetalAlpha) {

    if (waypoints.size() < 2) {
        throw std::runtime_error("At least 2 waypoints are required.");
    }
    if (samples_per_segment <= 0) {
        throw std::runtime_error("samples_per_segment must be positive.");
    }

    if (waypoints.size() == 2) {
        return connectLinear(waypoints, samples_per_segment);
    }
    if (waypoints.size() == 3) {
        return connectLinear(waypoints, samples_per_segment);
    }

    Path padded;
    padded.reserve(waypoints.size() + 2);
    padded.push_back(waypoints.front());

    for (const auto& p : waypoints) {
        padded.push_back(p);
    }

    padded.push_back(waypoints.back());

    constexpr int dense_multiplier = 10;
    const int dense_samples_per_segment =
        samples_per_segment * dense_multiplier;

    Path dense_path;
    dense_path.reserve(
        (waypoints.size() - 1) *
        static_cast<size_t>(dense_samples_per_segment) + 1
    );

    for (size_t seg = 0; seg + 3 < padded.size(); ++seg) {
        const JointVec& p0 = padded[seg];
        const JointVec& p1 = padded[seg + 1];
        const JointVec& p2 = padded[seg + 2];
        const JointVec& p3 = padded[seg + 3];

        for (int j = 0; j < dense_samples_per_segment; ++j) {
            const double u =
                static_cast<double>(j) /
                static_cast<double>(dense_samples_per_segment);

            dense_path.push_back(
                catmullRomPointGeneral(p0, p1, p2, p3, u, alpha)
            );
        }
    }

    dense_path.push_back(waypoints.back());

    const int output_samples =
        static_cast<int>(waypoints.size() - 1) * samples_per_segment + 1;

    return resampleByJointDistance(dense_path, output_samples);
}


Path connectPath(const Path& waypoints, Pathmode mode, int samples_per_segment) {
    switch (mode) {
        case Pathmode::Linear:
            return connectLinear(waypoints, samples_per_segment);
        case Pathmode::CatmullRomSpline:
            return connectCatmullRomSpline(waypoints, samples_per_segment, kCentripetalAlpha);
        default:
            throw std::runtime_error("Unsupported path mode.");
    }
}

TimedPath buildTimedReplayTrajectory(const Path& path, double dt) {
    if (path.empty()) {
        throw std::runtime_error("Path is empty.");
    }
    if (dt <= 0.0) {
        throw std::runtime_error("dt must be positive.");
    }

    TimedPath traj;
    traj.reserve(path.size());

    for (size_t k = 0; k < path.size(); ++k) {
        traj.push_back(TimedJointPoint{
            static_cast<double>(k) * dt,
            path[k]
        });
    }
    return traj;
}

struct ReplayCheckResult {
    bool ok{true};
    size_t bad_segment_idx{0};
    size_t bad_joint_idx{0};
    double required_steps{0.0};
    double actual_velocity{0.0};
    double vmax_limit{0.0};
    std::string message;
};

ReplayCheckResult checkReplayFeasibility(
    const Path& path,
    double dt,
    const std::array<double, 7>& vmax) {

    ReplayCheckResult result;

    if (path.size() < 2) {
        result.ok = false;
        result.message = "Path must contain at least 2 points.";
        return result;
    }
    if (dt <= 0.0) {
        result.ok = false;
        result.message = "dt must be positive.";
        return result;
    }

    for (size_t j = 0; j < 7; ++j) {
        if (vmax[j] <= 0.0) {
            result.ok = false;
            result.message = "vmax must be positive for every joint.";
            return result;
        }
    }

    // Velocity check: v[k] = (q[k + 1] - q[k]) / dt
    for (size_t seg = 0; seg + 1 < path.size(); ++seg) {
        for (size_t j = 0; j < 7; ++j) {
            const double dq = path[seg + 1][j] - path[seg][j];
            const double v = std::abs(dq) / dt;

            if (v > vmax[j] + 1e-12) {
                result.ok = false;
                result.bad_segment_idx = seg;
                result.bad_joint_idx = j;
                result.actual_velocity = v;
                result.vmax_limit = vmax[j];

                std::ostringstream oss;
                oss << "Replay rejected before execution: velocity limit exceeded. "
                    << "Segment " << seg << " -> " << (seg + 1)
                    << ", joint " << j
                    << ", implied velocity = " << v << " rad/s"
                    << ", vmax = " << vmax[j] << " rad/s. "
                    << "Increase samples_per_segment or reduce target spacing.";
                result.message = oss.str();
                return result;
            }
        }
    }



    return result;
}


Eigen::Matrix<double, 7, 7> buildDampingMatrixFromStiffness(
    const std::array<std::array<double, 7>, 7>& stiffness,
    double zeta = 1.0) {

    Eigen::Matrix<double, 7, 7> Dq = Eigen::Matrix<double, 7, 7>::Zero();
    for (size_t i = 0; i < 7; ++i) {
        const double kii = std::max(stiffness[i][i], 0.0);
        Dq(i, i) = 2.0 * zeta * std::sqrt(kii);
    }
    return Dq;
}

Eigen::Matrix<double, 7, 7> buildStiffnessMatrix(
    const std::array<std::array<double, 7>, 7>& stiffness) {

    Eigen::Matrix<double, 7, 7> Kq = Eigen::Matrix<double, 7, 7>::Zero();
    for (size_t i = 0; i < 7; ++i) {
        for (size_t j = 0; j < 7; ++j) {
            Kq(i, j) = stiffness[i][j];
        }
    }
    return Kq;
}

}  // namespace

int moveJ_path(
    franka::Robot& robot,
    const std::vector<std::array<double, 7>>& waypoints,
    int samples_per_segment,
    Pathmode path_mode,
    const std::array<std::array<double, 7>, 7>& stiffness,
    const std::array<double, 7>& vmax,
    const std::array<double, 7>& acc_max,  
    const std::function<void(const franka::RobotState&)>& state_callback,
    const std::function<double()>& slowdown_factor_callback,
    const std::function<bool()>& should_stop) {

    (void)acc_max;

    if (waypoints.empty()) {
        throw std::runtime_error("waypoints is empty.");
    }
    if (samples_per_segment <= 0) {
        throw std::runtime_error("samples_per_segment must be positive.");
    }

    franka::Model model = robot.loadModel();
    franka::RobotState initial_state = robot.readOnce();
    std::array<double, 7> q_start = initial_state.q;

    Path raw_path = arrayToPath(q_start, waypoints);
    Path connected_path = connectPath(raw_path, path_mode, samples_per_segment);
    TimedPath trajectory = buildTimedReplayTrajectory(connected_path, kDefaultDt);

    if (trajectory.size() < 2) {
        throw std::runtime_error("Generated trajectory is too short.");
    }

    {
        ReplayCheckResult precheck = checkReplayFeasibility(connected_path, kDefaultDt, vmax);
        if (!precheck.ok) {
            throw std::runtime_error(precheck.message);
        }
    }
    
    const Eigen::Matrix<double, 7, 7> Kq = buildStiffnessMatrix(stiffness);
    const Eigen::Matrix<double, 7, 7> Dq = buildDampingMatrixFromStiffness(stiffness, 1.0);


    //tolarnce parameters for runtime monitoring,can be tuned for better performance
    constexpr double q_error_abort_norm = 0.50;
    constexpr double q_error_hold_abort_time = 0.10;

    constexpr std::array<double, 7> q_error_abort_per_joint = {
        0.30, 0.30, 0.30, 0.30, 0.35, 0.35, 0.35
    };





    bool runtime_abort_requested = false;
    std::string runtime_abort_reason;

    double replay_time = 0.0;
    size_t sample_idx = 0;
    double violation_time = 0.0;



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

        const TimedJointPoint& ref = trajectory[sample_idx];

        Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> q_ref(ref.q.data());

        const Eigen::Matrix<double, 7, 1> q_error = q_ref - q;

        std::array<double, 7> coriolis_array = model.coriolis(robot_state);
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
        if (should_stop()) {
            std::array<double, 7> safe_tau{};
            for (size_t i = 0; i < 7; ++i) {
                safe_tau[i] = coriolis_array[i] - 2.0 * Dq(i, i) * robot_state.dq[i];
            }

            safe_tau = franka::limitRate(
                franka::kMaxTorqueRate,
                safe_tau,
                robot_state.tau_J_d
            );

            return franka::MotionFinished(franka::Torques(safe_tau));
        }


        Eigen::Matrix<double, 7, 1> tau_d = coriolis + Kq * q_error - Dq * dq;

        std::array<double, 7> tau_d_array{};
        Eigen::Map<Eigen::Matrix<double, 7, 1>>(tau_d_array.data()) = tau_d;

        tau_d_array = franka::limitRate(
            franka::kMaxTorqueRate,
            tau_d_array,
            robot_state.tau_J_d
        );

        const double q_error_norm = q_error.norm();

        bool violated = false;
        bool norm_violated = false;
        int violated_joint = -1;
        double violated_joint_error = 0.0;

        if (q_error_norm > q_error_abort_norm) {
            violated = true;
            norm_violated = true;
        }

        for (size_t i = 0; i < 7; ++i) {
            const double joint_error = std::abs(q_error(i));
            if (joint_error > q_error_abort_per_joint[i]) {
                violated = true;
                violated_joint = static_cast<int>(i);
                violated_joint_error = joint_error;
                break;
            }
        }

        if (violated) {
            violation_time += dt;

            if (violation_time >= q_error_hold_abort_time && !runtime_abort_requested) {
                runtime_abort_requested = true;

                std::ostringstream oss;
                oss << "Runtime abort: joint tracking error too large. "
                    << "q_error_norm = " << q_error_norm
                    << ", norm threshold = " << q_error_abort_norm;

                if (norm_violated) {
                    oss << ", norm violated";
                }

                if (violated_joint >= 0) {
                    oss << ", joint = " << violated_joint
                        << ", joint error = " << violated_joint_error
                        << ", joint threshold = "
                        << q_error_abort_per_joint[static_cast<size_t>(violated_joint)];
                }

                oss << ".";
                runtime_abort_reason = oss.str();
            }
        } else {
            violation_time = 0.0;
        }
        if (runtime_abort_requested) {
            std::array<double, 7> safe_tau{};
            for (size_t i = 0; i < 7; ++i) {
                safe_tau[i] = coriolis_array[i] - 2.0 * Dq(i, i) * robot_state.dq[i];
            }

            safe_tau = franka::limitRate(
                franka::kMaxTorqueRate,
                safe_tau,
                robot_state.tau_J_d
            );

        return franka::MotionFinished(franka::Torques(safe_tau));
    }



        const double traj_T = trajectory.back().t;

        if (replay_time >= traj_T) {
           return franka::MotionFinished(franka::Torques(tau_d_array));
        }

        return franka::Torques(tau_d_array);
    };

    robot.control(torque_callback);


    if (runtime_abort_requested) {
        std::cerr << runtime_abort_reason << std::endl;
        throw std::runtime_error(runtime_abort_reason);
    }

    std::cout << "Motion finished." << std::endl;
    return 0;
}
