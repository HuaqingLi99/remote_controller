#pragma once

#include <array>
#include <functional>

#include <franka/robot.h>

int trackJ(
    franka::Robot& robot,
    int command_port,
    double stream_hz,
    const std::array<std::array<double, 7>, 7>& stiffness,
    const std::function<void(const franka::RobotState&)>& state_callback,
    const std::function<bool()>& should_stop);