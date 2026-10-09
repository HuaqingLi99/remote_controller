#include "gripperhome.h"

#include <chrono>
#include <iostream>
#include <thread>

#include <franka/exception.h>
#include <franka/gripper.h>


/**
 * Direct Franka gripper homing helper.
 *
 * Homing calibrates the gripper width range and should be run when the gripper
 * needs to re-establish its mechanical reference.
 */

int gripperHome(franka::Gripper& gripper) {
    try {
        bool success = gripper.homing();

        if (!success) {
            std::cerr << "Gripper homing failed." << std::endl;
            return 1;
        }

        std::cout << "Gripper homing finished successfully." << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error during gripper homing: " << e.what() << std::endl;
        return 1;
    }
}
