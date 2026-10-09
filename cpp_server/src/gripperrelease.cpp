#include "gripperrelease.h"

#include <iostream>
#include <franka/gripper.h>
#include <franka/exception.h>

/**
 * Direct Franka gripper release/open helper.
 *
 * Opens the gripper to state.max_width with the requested speed in m/s and
 * reports the final measured width in meters.
 */



ReleaseResult gripperRelease(franka::Gripper& gripper,
                             double speed) {
    ReleaseResult result{};
    result.success = false;
    result.was_holding = false;
    result.final_width = 0.0;

    try {
        franka::GripperState state = gripper.readOnce();

        result.was_holding = state.is_grasped;

        std::cout << "[gripperRelease] current width=" << state.width
                  << ", max width=" << state.max_width
                  << ", is_grasped=" << state.is_grasped
                  << std::endl;

        if (speed <= 0.0) {
            std::cerr << "[gripperRelease] Invalid speed." << std::endl;
            return result;
        }



        bool opened = gripper.move(state.max_width, speed);
        if (!opened) {
            std::cerr << "[gripperRelease] Failed to open gripper." << std::endl;
            return result;
        }

        franka::GripperState final_state = gripper.readOnce();
        result.final_width = final_state.width;
        result.success = true;

        std::cout << "[gripperRelease] final width=" << final_state.width
                  << ", is_grasped=" << final_state.is_grasped
                  << std::endl;

        return result;
    } catch (const std::exception& e) {
        std::cerr << "[gripperRelease] Error: " << e.what() << std::endl;
        return result;
    }
}
