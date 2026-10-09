
#pragma once

#include<iostream>
#include <array>
#include <cmath>
#include <functional>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include <functional>




/**
 * Execute one blocking joint-space impedance move.
 *
 * Args:
 * - q_d: 7 joint targets in radians
 * - stiffness: 7x7 joint stiffness matrix
 * - vmax: per-joint velocity limits in rad/s
 * - acc_max: per-joint acceleration limits in rad/s^2
 * - state_callback: called from the control loop with each robot state
 * - slowdown_factor_callback: checked during control for runtime replay speed
 * - cancel_callback: checked during control for stop requests
 *
 * Returns:
 * - 0 on success
 * - throws or returns an RPC error code through Robotmanager on failure
 */
int moveJ(
    franka::Robot& robot,
    const std::array<double, 7>& q_d,
    const std::array<std::array<double, 7>, 7>& stiffness,
    const std::array<double, 7>& vmax,
    const std::array<double, 7>& acc_max,
    const std::function<void(const franka::RobotState&)>& state_callback,
    const std::function<double()>& slowdown_factor_callback,
    const std::function<bool()>& cancel_callback);
