#pragma once
#include <franka/gripper.h>

/**
 * Result returned by one direct Franka gripper grasp attempt.
 */
struct GraspResult {
    bool success;
    bool width_too_large;
    bool is_grasped;
    double current_width;
};


/**
 * Execute a blocking Franka gripper grasp.
 *
 * Args:
 * - width: target grasp width in meters
 * - speed: gripper closing speed in m/s
 * - force: grasp force in newtons
 * - epsilon_inner: accepted inner width tolerance in meters
 * - epsilon_outer: accepted outer width tolerance in meters
 *
 * Returns:
 * - GraspResult with success flags and the measured current width in meters
 */
GraspResult graspO(franka::Gripper& gripper, double width, double speed, double force, double epsilon_inner, double epsilon_outer);
