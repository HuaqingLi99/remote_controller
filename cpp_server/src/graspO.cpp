#include "graspO.h"

#include <chrono>
#include <iostream>
#include <thread>

#include <franka/exception.h>
#include <franka/gripper.h>

/**
 * Direct Franka gripper grasp helper.
 *
 * This file wraps libfranka::Gripper::grasp and normalizes the result into a
 * GraspResult that Robotmanager can translate into gripper status and RPC
 * result codes. Width and epsilon values are meters, speed is m/s, force is N.
 */

GraspResult graspO(
    franka::Gripper& gripper,
    double width,
    double speed,
    double force,
    double epsilon_inner,
    double epsilon_outer) {
    GraspResult result{};
    result.success = false;
    result.width_too_large = false;
    result.is_grasped = false;
    result.current_width = 0.0;

    std::cout << "[graspO] Request received:"
              << " width=" << width
              << " speed=" << speed
              << " force=" << force
              << " epsilon_inner=" << epsilon_inner
              << " epsilon_outer=" << epsilon_outer
              << std::endl;

    if (width <= 0.0 || speed <= 0.0 || force <= 0.0 ||
        epsilon_inner < 0.0 || epsilon_outer < 0.0) {
        std::cout << "[graspO] Invalid parameters." << std::endl;
        return result;
    }

    franka::GripperState state = gripper.readOnce();
    result.current_width = state.width;

    std::cout << "[graspO] Current width: " << state.width
              << ", max width: " << state.max_width
              << ", is_grasped: " << state.is_grasped
              << std::endl;

    if (state.max_width < width) {
        std::cout << "[graspO] Target width is too large." << std::endl;
        result.width_too_large = true;
        return result;
    }

    if (!gripper.move(state.max_width, speed)) {
        std::cout << "[graspO] Failed to open gripper." << std::endl;
        return result;
    }

    std::cout << "[graspO] Gripper opened, start grasp." << std::endl;

    result.success = gripper.grasp(width, speed, force, epsilon_inner, epsilon_outer);

    std::cout << "[graspO] grasp() returned: " << result.success << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    state = gripper.readOnce();
    result.current_width = state.width;
    result.is_grasped = state.is_grasped;

    std::cout << "[graspO] Final width: " << state.width
              << ", is_grasped: " << state.is_grasped
              << std::endl;

    return result;
}
