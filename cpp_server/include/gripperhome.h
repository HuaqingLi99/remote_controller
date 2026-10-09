#pragma once
#include <franka/gripper.h>



/**
 * Execute Franka gripper homing.
 *
 * Returns:
 * - 0 when homing succeeds
 * - 1 when libfranka reports failure or throws
 */
int gripperHome(franka::Gripper& gripper);
