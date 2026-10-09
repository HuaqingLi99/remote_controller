#include <iostream>
#include <string>
#include <thread>

#include <franka/gripper.h>
#include <franka/robot.h>
#include <xmlrpc-c/registry.hpp>
#include <xmlrpc-c/server_abyss.hpp>

#include <robot_manager.h>
#include <rpc_methods.h>
#include <robot_init.h>

/**
 * Server entrypoint.
 *
 * Creates the Franka robot/gripper connections, registers XML-RPC methods, and
 * starts worker threads for arm commands, gripper commands, UDP state streaming,
 * and idle-state polling.
 */

int main(int argc, char** argv) {
    try {
        if (argc < 4) {
            std::cerr << "Usage: " << argv[0] << " <robot_ip> <gripper_ip> <xmlrpc_port>" << std::endl;
            return 1;
        }

        std::string robot_ip = argv[1];
        std::string gripper_ip = argv[2];
        int xmlrpc_port = std::stoi(argv[3]);

        franka::Robot robot(robot_ip);
        franka::Gripper gripper(gripper_ip);
        Robotmanager robot_manager(robot, gripper);
        Robot_init robot_init(robot_manager);
        if (!robot_init.init(robot_ip, xmlrpc_port)) {
            std::cerr << "Robot initialization failed." << std::endl;
            return 1;
        }

        xmlrpc_c::registry registry;
        register_methods(registry, robot_manager);

        std::thread arm_worker(&Robotmanager::worker_arm_thread, &robot_manager);
        std::thread gripper_worker(&Robotmanager::worker_gripper_thread, &robot_manager);
        std::thread udp_worker(&Robotmanager::worker_udp_thread, &robot_manager);
        std::thread state_poll_worker(&Robotmanager::worker_robot_state_poll_thread,&robot_manager);
        
        xmlrpc_c::serverAbyss server(
            xmlrpc_c::serverAbyss::constrOpt()
                .registryP(&registry)
                .portNumber(xmlrpc_port));

 
        std::cout << "Call initSession() before expecting UDP streaming." << std::endl;

        server.run();

        robot_manager.stop_worker.store(true);
        robot_manager.queue_arm_cv.notify_one();
        robot_manager.queue_gripper_cv.notify_one();

        arm_worker.join();
        gripper_worker.join();
        udp_worker.join();
        state_poll_worker.join();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
