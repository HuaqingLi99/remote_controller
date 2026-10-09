#pragma once

#include <array>
#include <functional>
#include <vector>

#include <franka/robot.h>
#include <franka/robot_state.h>

#include "moveJ_path.h"

/**
 * Execute one blocking Cartesian waypoint path.
 *
 * Args:
 * - waypoints: Nx16 row-major 4x4 poses in base/world frame; current O_T_EE is
 *   prepended in cpp
 * - samples_per_segment: number of base replay samples per path segment
 * - path_mode: linear or Catmull-Rom interpolation
 * - stiffness: 6x6 Cartesian stiffness matrix
 * - vmax_linear: linear velocity limit in m/s, checked before execution
 * - acc_max_linear: accepted for API compatibility, not used by replay timing
 * - vmax_angular: angular velocity limit in rad/s, checked before execution
 * - acc_max_angular: accepted for API compatibility, not used by replay timing
 * - nullspace_stiffness: posture-holding nullspace stiffness
 * - state_callback: called from the control loop with each robot state
 * - slowdown_factor_callback: checked during control for runtime replay speed
 * - should_stop: checked during control for stop requests
 *
 * Returns:
 * - 0 on success
 * - throws or returns an RPC error code through Robotmanager on failure
 */
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
    const std::function<bool()>& should_stop);
