#include "trackC.h"
#include "trackC_udp_receiver.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <franka/duration.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

namespace {

Eigen::Matrix4d arrayToMatrix4d(const std::array<double, 16>& T) {
    Eigen::Matrix4d M;
    for (int i = 0; i < 16; ++i) {
        M(i / 4, i % 4) = T[i];
    }
    return M;
}

std::array<double, 16> matrix4dToRowMajorArray(const Eigen::Matrix4d& T) {
    std::array<double, 16> out{};
    for (int i = 0; i < 16; ++i) {
        out[i] = T(i / 4, i % 4);
    }
    return out;
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

int trackC(
    franka::Robot& robot,
    int command_port,
    double stream_hz,
    const std::array<std::array<double, 6>, 6>& stiffness,
    double nullspace_stiffness,
    const std::function<void(const franka::RobotState&)>& state_callback,
    const std::function<bool()>& should_stop) {

    if (!std::isfinite(stream_hz) || stream_hz <= 0.0) {
        throw std::runtime_error("trackC stream_hz must be positive.");
    }

    franka::Model model = robot.loadModel();
    franka::RobotState initial_state = robot.readOnce();

    const Eigen::Matrix4d T_start =
        Eigen::Map<const Eigen::Matrix4d>(initial_state.O_T_EE.data());
    const std::array<double, 16> T_start_row_major =
        matrix4dToRowMajorArray(T_start);

    Eigen::Map<const Eigen::Matrix<double, 7, 1>> q_start_joint(initial_state.q.data());

    const Eigen::Matrix<double, 6, 6> Kx =
        buildCartesianStiffnessMatrix(stiffness);
    const Eigen::Matrix<double, 6, 6> Dx =
        buildCartesianDampingMatrixFromStiffness(stiffness, 1.0);

    TrackCUdpReceiver receiver(command_port, stream_hz, T_start_row_major);
    receiver.start();

    std::array<double, 16> last_valid_T_ref = T_start_row_major;

    auto torque_callback =
        [&](const franka::RobotState& robot_state,
            franka::Duration /*duration*/) -> franka::Torques {

        state_callback(robot_state);

        TrackCTargetSnapshot target{};
        if (receiver.getLatestTarget(target)) {
            last_valid_T_ref = target.T_ref;
        }

        Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

        Eigen::Matrix4d T =
            Eigen::Map<const Eigen::Matrix4d>(robot_state.O_T_EE.data());
        Eigen::Vector3d p = T.block<3, 1>(0, 3);
        Eigen::Matrix3d R = T.block<3, 3>(0, 0);

        Eigen::Matrix4d T_ref = arrayToMatrix4d(last_valid_T_ref);
        Eigen::Vector3d p_d = T_ref.block<3, 1>(0, 3);
        Eigen::Matrix3d R_d = T_ref.block<3, 3>(0, 0);

        Eigen::Matrix<double, 6, 1> error;
        error.head<3>() = p_d - p;
        error.tail<3>() = rotationError(R, R_d);

        std::array<double, 42> jacobian_array =
            model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
        Eigen::Map<const Eigen::Matrix<double, 6, 7>> J(jacobian_array.data());

        const Eigen::Matrix<double, 6, 1> dx = J * dq;
        const Eigen::Matrix<double, 6, 1> derror = -dx;

        std::array<double, 7> coriolis_array = model.coriolis(robot_state);
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());

        auto finish_with_damping = [&]() -> franka::Torques {
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
        };

        if (should_stop()) {
            return finish_with_damping();
        }

        if (receiver.hasFault()) {
            return finish_with_damping();
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

    std::cout << "trackC finished." << std::endl;
    return 0;
}
