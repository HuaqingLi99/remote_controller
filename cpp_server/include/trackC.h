#pragma once

#include <array>
#include <functional>

#include <franka/robot.h>

int trackC(
    franka::Robot& robot,
    int command_port,
    double stream_hz,
    const std::array<std::array<double, 6>, 6>& stiffness,
    double nullspace_stiffness,
    const std::function<void(const franka::RobotState&)>& state_callback,
    const std::function<bool()>& should_stop);
