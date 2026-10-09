#pragma once

#include <array>
#include <functional>

#include <franka/robot.h>

/**
 * Execute one blocking Cartesian impedance move to a single pose.
 *
 * Args:
 * - T_d: row-major flattened 4x4 target pose in base/world frame
 * - stiffness: 6x6 Cartesian stiffness matrix
 * - vmax_linear: linear velocity limit in m/s
 * - acc_max_linear: linear acceleration limit in m/s^2
 * - vmax_angular: angular velocity limit in rad/s
 * - acc_max_angular: angular acceleration limit in rad/s^2
 * - nullspace_stiffness: posture-holding nullspace stiffness
 * - state_callback: called from the control loop with each robot state
 * - slowdown_factor_callback: checked during control for runtime replay speed
 * - cancel_callback: checked during control for stop requests
 *
 * Returns:
 * - 0 on success
 * - throws or returns an RPC error code through Robotmanager on failure
 */
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
    const std::function<bool()>& cancel_callback);
