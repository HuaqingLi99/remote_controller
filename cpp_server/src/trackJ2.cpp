#include "trackJ2.h"
#include "trackJ_udp_receiver.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <utility>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

namespace {

using JointVec = std::array<double, 7>;

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

double safeDt(franka::Duration duration) {
    const double dt = duration.toSec();
    if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.1) {
        return 0.001;
    }
    return dt;
}

std::pair<double, double> safeFilterParams(
    const std::function<std::pair<double, double>()>& callback) {

    std::pair<double, double> params = callback();
    if (!std::isfinite(params.first) || params.first <= 0.0) {
        params.first = 10.0;
    }
    if (!std::isfinite(params.second) || params.second <= 0.0) {
        params.second = 1.0;
    }
    return params;
}

}  // namespace

int trackJ2(
    franka::Robot& robot,
    int command_port,
    double stream_hz,
    const std::array<std::array<double, 7>, 7>& stiffness,
    const std::function<std::pair<double, double>()>& filter_params_callback,
    const std::function<void(const franka::RobotState&)>& state_callback,
    const std::function<bool()>& should_stop) {

    if (!std::isfinite(stream_hz) || stream_hz <= 0.0) {
        throw std::runtime_error("trackJ2 stream_hz must be positive.");
    }

    franka::Model model = robot.loadModel();
    franka::RobotState initial_state = robot.readOnce();
    JointVec q_start = initial_state.q;

    const Eigen::Matrix<double, 7, 7> Kq =
        buildStiffnessMatrix(stiffness);
    const Eigen::Matrix<double, 7, 7> Dq =
        buildDampingMatrixFromStiffness(stiffness, 1.0);

    TrackJUdpReceiver receiver(command_port, stream_hz, q_start);
    receiver.start();

    JointVec last_valid_q_target = q_start;
    Eigen::Matrix<double, 7, 1> q_ref =
        Eigen::Map<const Eigen::Matrix<double, 7, 1>>(q_start.data());
    Eigen::Matrix<double, 7, 1> dq_ref =
        Eigen::Matrix<double, 7, 1>::Zero();

    auto torque_callback =
        [&](const franka::RobotState& robot_state,
            franka::Duration duration) -> franka::Torques {

        state_callback(robot_state);

        TrackJTargetSnapshot target{};
        if (receiver.getLatestTarget(target)) {
            last_valid_q_target = target.q_ref;
        }

        Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> q_target(last_valid_q_target.data());

        std::array<double, 7> coriolis_array = model.coriolis(robot_state);
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());

        auto finish_with_damping = [&]() -> franka::Torques {
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
        };

        if (should_stop()) {
            return finish_with_damping();
        }

        if (receiver.hasFault()) {
            return finish_with_damping();
        }

        const double dt = safeDt(duration);
        const std::pair<double, double> params =
            safeFilterParams(filter_params_callback);
        const double omega_n = params.first;
        const double zeta = params.second;

        const Eigen::Matrix<double, 7, 1> ddq_ref =
            omega_n * omega_n * (q_target - q_ref)
            - 2.0 * zeta * omega_n * dq_ref;

        dq_ref += ddq_ref * dt;
        q_ref += dq_ref * dt;

        const Eigen::Matrix<double, 7, 1> q_error = q_ref - q;
        const Eigen::Matrix<double, 7, 1> dq_error = dq_ref - dq;

        Eigen::Matrix<double, 7, 1> tau_d =
            coriolis + Kq * q_error + Dq * dq_error;

        std::array<double, 7> tau_d_array{};
        Eigen::Map<Eigen::Matrix<double, 7, 1>>(tau_d_array.data()) = tau_d;

        tau_d_array = franka::limitRate(
            franka::kMaxTorqueRate,
            tau_d_array,
            robot_state.tau_J_d
        );

        return franka::Torques(tau_d_array);
    };

    try {
        robot.control(torque_callback);
    } catch (...) {
        receiver.stop();
        throw;
    }

    receiver.stop();

    if (receiver.hasFault()) {
        throw std::runtime_error(receiver.faultReason());
    }

    std::cout << "trackJ2 finished." << std::endl;
    return 0;
}
