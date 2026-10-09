#pragma once

#include <string>

class Robotmanager;

/**
 * Startup helper for checking robot/gripper connectivity and printing server
 * information before XML-RPC starts serving requests.
 */
class Robot_init {
public:
    explicit Robot_init(Robotmanager& manager)
        : manager_(manager) {}

    /**
     * Run startup checks for robot, gripper, and XML-RPC port reporting.
     *
     * Args:
     * - robot_ip: Franka robot IP address used for display/logging
     * - xmlrpc_port: XML-RPC server port
     *
     * Returns:
     * - true when all startup checks succeed
     */
    bool init(const std::string& robot_ip, int xmlrpc_port);

private:
    Robotmanager& manager_;

    bool connectRobot();
    bool connectGripper();
    bool printPort(const std::string& robot_ip, int xmlrpc_port);
};
