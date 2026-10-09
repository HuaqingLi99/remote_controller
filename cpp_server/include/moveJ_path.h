#pragma once

#include <array>
#include <functional>
#include <vector>

#include <franka/robot.h>
#include <franka/robot_state.h>

enum class Pathmode {
    Linear,
    CatmullRomSpline
};

/**
 * Execute one blocking joint-space waypoint path.
 *
 * Args:
 * - waypoints: Nx7 joint waypoints in radians; current q is prepended in cpp
 * - samples_per_segment: number of base replay samples per path segment
 * - path_mode: linear or Catmull-Rom interpolation
 * - stiffness: 7x7 joint stiffness matrix
 * - vmax: per-joint velocity limits in rad/s, checked before execution
 * - acc_max: accepted for task/API compatibility, not used by replay timing
 * - state_callback: called from the control loop with each robot state
 * - slowdown_factor_callback: checked during control for runtime replay speed
 * - cancel_callback: checked during control for stop requests
 *
 * Returns:
 * - 0 on success
 * - throws or returns an RPC error code through Robotmanager on failure
 */
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
    const std::function<bool()>& cancel_callback);
