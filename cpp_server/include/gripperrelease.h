#pragma once
#include <franka/gripper.h>
#include <franka/exception.h>


/**
 * Result returned by one direct Franka gripper release/open command.
 */
struct ReleaseResult {
    bool success;
    bool was_holding;
    double final_width;
};

/**
 * Open the gripper to its maximum width.
 *
 * Args:
 * - speed: gripper opening speed in m/s
 *
 * Returns:
 * - ReleaseResult with success flag, previous grasp state, and final width in m
 */
ReleaseResult gripperRelease(franka::Gripper& gripper,double speed);
