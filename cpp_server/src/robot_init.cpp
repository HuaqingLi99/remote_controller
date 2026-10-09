#include "robot_init.h"

#include "robot_manager.h"

#include <stdexcept>

/**
 * Robot and gripper startup checks.
 *
 * This file performs simple blocking readOnce checks through Robotmanager so
 * startup fails early when the robot or gripper connection is not usable.
 */

bool Robot_init::init(const std::string& robot_ip, int xmlrpc_port) {
    if (!connectRobot()) {
        return false;
    }

    if (!connectGripper()) {
        return false;
    }

    if (!printPort(robot_ip, xmlrpc_port)) {
        return false;
    }

    return true;
}






bool Robot_init::connectRobot() {
    try{
        franka::RobotState state = manager_.readRobotState();
        std::cout << "Robot connected. Current joint positions: ";
        for (size_t i = 0; i < 7; ++i) {
            std::cout << state.q[i] << " ";
        }
        std::cout << std::endl;
    }catch(const std::exception& e){
        std::cerr << "Failed to connect to robot: " << e.what() << std  ::endl;
        return false;
    }
    return true;
}


bool Robot_init::connectGripper() {
    try{
        franka::GripperState state = manager_.readGripperState();
        std::cout << "Gripper connected. Current width: " << state.width
                  << ", is_grasped: " << state.is_grasped
                  << std::endl;
    }catch(const std::exception& e){
        std::cerr << "Failed to connect to gripper: " << e.what() << std  ::endl;
        return false;
    }
    return true;
}   


bool Robot_init::printPort(const std::string& robot_ip, int xmlrpc_port)
 {
    try{

        std::cout << "Robot IP: " << robot_ip << std::endl;
        std::cout << "XML-RPC server port: " << xmlrpc_port << std::endl;

        
    }catch(const std::exception& e){
        std::cerr << "Failed to read robot/gripper state: " << e.what() << std  ::endl;
        return false;
    }
    return true;
}
