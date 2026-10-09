#include "trackJ.h"
#include "trackJ_udp_receiver.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>

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

}  // namespace

int trackJ(
    franka::Robot& robot,
    int command_port,
    double stream_hz,
    const std::array<std::array<double, 7>, 7>& stiffness,
    const std::function<void(const franka::RobotState&)>& state_callback,
    const std::function<bool()>& should_stop) {

    if (!std::isfinite(stream_hz) || stream_hz <= 0.0) {
        throw std::runtime_error("trackJ stream_hz must be positive.");
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

    JointVec last_valid_q_ref = q_start;

    auto torque_callback =
        [&](const franka::RobotState& robot_state,
            franka::Duration /*duration*/) -> franka::Torques {

        state_callback(robot_state);

        TrackJTargetSnapshot target{};
        if (receiver.getLatestTarget(target)) {
            last_valid_q_ref = target.q_ref;
        }

        Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> q_ref(last_valid_q_ref.data());

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

        const Eigen::Matrix<double, 7, 1> q_error = q_ref - q;

        Eigen::Matrix<double, 7, 1> tau_d =
            coriolis + Kq * q_error - Dq * dq;

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

    std::cout << "trackJ finished." << std::endl;
    return 0;
}
