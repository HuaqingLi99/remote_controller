#include "robot_manager.h"
#include "moveJ_path.h"
#include "graspO.h"
#include "gripperhome.h"
#include "gripperrelease.h"
#include "moveJ.h"
#include "move_cartesian.h"
#include "udp_state_publisher.h"
#include "trackJ.h"
#include "trackJ2.h"
#include "trackC.h"
#include "trackC2.h"
#include <stdexcept>
#include <algorithm>
#include <cmath>




#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

/**
 * Central runtime coordinator for robot, gripper, queues, and UDP state.
 *
 * Direct commands enter execution immediately if the arm/gripper is idle.
 * Queued commands are consumed serially by worker threads. Robot state is cached
 * from active control callbacks and idle polling, then published over UDP as a
 * best-effort state stream.
 */

namespace {
int try_enter_arm_motion(ArmStatus& arm_status) {
    // Direct arm commands use this gate so they fail fast instead of fighting
    // an active controller. Queued commands wait in worker_arm_thread().
    arm_state current = arm_status.state.load();
    while (true) {
        if (current == arm_state::MOVING) {
            return RPC_DENIED_MOVING;
        }
        if (current == arm_state::ERROR) {
            return RPC_DENIED_ERROR;
        }
        if (arm_status.state.compare_exchange_weak(current, arm_state::MOVING)) {
            return RPC_OK;
        }
    }
}

int try_enter_gripper_motion(GripperStatus& gripper_status) {
    // Same direct-command gate for the gripper worker.
    gripper_state current = gripper_status.state.load();
    while (true) {
        if (current == gripper_state::MOVING) {
            return RPC_DENIED_MOVING;
        }
        if (current == gripper_state::ERROR) {
            return RPC_DENIED_ERROR;
        }
        if (gripper_status.state.compare_exchange_weak(current, gripper_state::MOVING)) {
            return RPC_OK;
        }
    }
}

std::chrono::steady_clock::duration hzToPeriod(int hz) {
    const int safe_hz = std::max(1, hz);
    return std::chrono::microseconds(
        std::max<std::int64_t>(1, 1000000LL / safe_hz)
    );
}



bool nearlyEqual(double a, double b, double eps) {
    return std::fabs(a - b) <= eps;
}
//check wheter if the state change or not
bool robotStateChanged(const robot_state_udp& a, const robot_state_udp& b) {
    constexpr double kQEps = 1e-5;
    constexpr double kDqEps = 1e-4;
    constexpr double kForceEps = 1e-3;

    for (size_t i = 0; i < 7; ++i) {
        if (!nearlyEqual(a.q[i], b.q[i], kQEps)) {
            return true;
        }
        if (!nearlyEqual(a.dq[i], b.dq[i], kDqEps)) {
            return true;
        }
    }

    for (size_t i = 0; i < 6; ++i) {
        if (!nearlyEqual(a.K_F_ext_hat[i], b.K_F_ext_hat[i], kForceEps)) {
            return true;
        }
    }

    if (a.arm_state != b.arm_state) {
        return true;
    }

    if (a.gripper_state != b.gripper_state) {
        return true;
    }

    return false;
}
}



Robotmanager::Robotmanager(franka::Robot& robot, franka::Gripper& gripper)
    : robot_(robot), gripper_(gripper) {}

void Robotmanager::enqueueMoveJ(const MoveJTask& task) {
    {
        std::lock_guard<std::mutex> lock(queue_arm_mutex);
        // Queue APIs only store work and wake the worker. The worker later
        // applies the same motion/error-state gate before execution.
        ArmTask arm_task{};
        arm_task.type = ArmCommandType::MOVEJ;
        arm_task.movej_task = task;
        arm_queue.push(arm_task);
    }
    queue_arm_cv.notify_one();
}


void Robotmanager::enqueueMoveCartesian(const MoveCartesianTask& task) {
    {
        std::lock_guard<std::mutex> lock(queue_arm_mutex);
        ArmTask arm_task{};
        arm_task.type = ArmCommandType::MOVECART;
        arm_task.movecart_task = task;
        arm_queue.push(arm_task);
    }
    queue_arm_cv.notify_one();
}


void Robotmanager::enqueueGripper(const GripperTask& task) {
    {
        std::lock_guard<std::mutex> lock(queue_gripper_mutex);
        gripper_queue.push(task);
    }
    queue_gripper_cv.notify_one();
}

void Robotmanager::enqueueMoveJPath(const MoveJPathTask& task) {
    {
        std::lock_guard<std::mutex> lock(queue_arm_mutex);
        ArmTask arm_task{};
        arm_task.type = ArmCommandType::MOVEJ_PATH;
        arm_task.movej_path_task = task;
        arm_queue.push(arm_task);
    }
    queue_arm_cv.notify_one();
}

int Robotmanager::executeMoveJPath(const MoveJPathTask& task) {
    try {
        int gate = try_enter_arm_motion(arm_status);
        if (gate != RPC_OK) {
            if (gate == RPC_DENIED_MOVING) {
                std::cerr << "Arm is currently moving. Cannot execute direct moveJPath." << std::endl;
            } else if (gate == RPC_DENIED_ERROR) {
                std::cerr << "Arm is in error state. Cannot execute direct moveJPath." << std::endl;
            }
            return gate;
        }
        arm_cancel_requested.store(false, std::memory_order_release);


        int result = 0;
        {
            std::lock_guard<std::mutex> robot_lock(robot_io_mutex);
            result = moveJ_path(
                robot_,
                task.waypoints,
                task.samples_per_segment,
                task.path_mode,
                task.stiffness,
                task.vmax,
                task.acc_max,
                [this](const franka::RobotState& state) {
                    this->updateRobotState(state);
                },
                [this]() {
                    return this->getMotionSlowdownFactor();
                },
                [this]() {
                    return this->isArmCancelRequested();
                });
        }


        if (result == 0) {
            arm_status.state.store(arm_state::IDLE);
            return RPC_OK;
        } else {
            arm_status.state.store(arm_state::ERROR);
            return RPC_EXECUTION_FAILED;
        }
    } catch (const std::exception& e) {
        std::cerr << "Error executing direct moveJPath: " << e.what() << std::endl;
        arm_status.state.store(arm_state::ERROR);
        return RPC_EXECUTION_FAILED;
    }
}


int Robotmanager::executeMoveJ(const MoveJTask& task) {
    try {
        int gate = try_enter_arm_motion(arm_status);
        if (gate != RPC_OK) {
            if (gate == RPC_DENIED_MOVING) {
                std::cerr << "Arm is currently moving. Cannot execute direct moveJ." << std::endl;
            } else if (gate == RPC_DENIED_ERROR) {
                std::cerr << "Arm is in error state. Cannot execute direct moveJ." << std::endl;
            }
            return gate;
        }
        arm_cancel_requested.store(false, std::memory_order_release);

        int result = 0;
        {
            std::lock_guard<std::mutex> robot_lock(robot_io_mutex);
            result = moveJ(
                robot_,
                task.q_d,
                task.stiffness,
                task.vmax,
                task.acc_max,
                [this](const franka::RobotState& state) {
                    this->updateRobotState(state);
                },
                [this]() {
                    return this->getMotionSlowdownFactor();
                },
                [this]() {
                    return this->isArmCancelRequested();
                }
            );
        }


        if (result == 0) {
            arm_status.state.store(arm_state::IDLE);
            return RPC_OK;
        } else {
            arm_status.state.store(arm_state::ERROR);
            return RPC_EXECUTION_FAILED;
        }
      
        return result;
    } catch (const std::exception& e) {
        std::cerr << "Error executing direct moveJ: " << e.what() << std::endl;
        arm_status.state.store(arm_state::ERROR);
        return RPC_EXECUTION_FAILED;
    }
}

int Robotmanager::startTrackJ(const TrackJTask& task) {
    try {
        if (task.command_port <= 0 || task.command_port > 65535) {
            std::cerr << "Invalid trackJ command port." << std::endl;
            return RPC_EXECUTION_FAILED;
        }
        if (!std::isfinite(task.stream_hz) || task.stream_hz <= 0.0) {
            std::cerr << "Invalid trackJ stream frequency." << std::endl;
            return RPC_EXECUTION_FAILED;
        }

        int gate = try_enter_arm_motion(arm_status);
        if (gate != RPC_OK) {
            if (gate == RPC_DENIED_MOVING) {
                std::cerr << "Arm is currently moving. Cannot start trackJ." << std::endl;
            } else if (gate == RPC_DENIED_ERROR) {
                std::cerr << "Arm is in error state. Cannot start trackJ." << std::endl;
            }
            return gate;
        }

        arm_cancel_requested.store(false, std::memory_order_release);

        std::thread([this, task]() {
            int result = 0;

            try {
                {
                    std::lock_guard<std::mutex> robot_lock(robot_io_mutex);
                    result = trackJ(
                        robot_,
                        task.command_port,
                        task.stream_hz,
                        task.stiffness,
                        [this](const franka::RobotState& state) {
                            this->updateRobotState(state);
                        },
                        [this]() {
                            return this->isArmCancelRequested();
                        });
                }

                if (result == 0) {
                    arm_status.state.store(arm_state::IDLE);
                } else {
                    arm_status.state.store(arm_state::ERROR);
                }
            } catch (const std::exception& e) {
                std::cerr << "Error executing trackJ: " << e.what() << std::endl;
                arm_status.state.store(arm_state::ERROR);
            }
        }).detach();

        return RPC_OK;
    } catch (const std::exception& e) {
        std::cerr << "Failed to start trackJ: " << e.what() << std::endl;
        arm_status.state.store(arm_state::ERROR);
        return RPC_EXECUTION_FAILED;
    }
}

int Robotmanager::startTrackJ2(const TrackJTask& task) {
    try {
        if (task.command_port <= 0 || task.command_port > 65535) {
            std::cerr << "Invalid trackJ2 command port." << std::endl;
            return RPC_EXECUTION_FAILED;
        }
        if (!std::isfinite(task.stream_hz) || task.stream_hz <= 0.0) {
            std::cerr << "Invalid trackJ2 stream frequency." << std::endl;
            return RPC_EXECUTION_FAILED;
        }

        int gate = try_enter_arm_motion(arm_status);
        if (gate != RPC_OK) {
            if (gate == RPC_DENIED_MOVING) {
                std::cerr << "Arm is currently moving. Cannot start trackJ2." << std::endl;
            } else if (gate == RPC_DENIED_ERROR) {
                std::cerr << "Arm is in error state. Cannot start trackJ2." << std::endl;
            }
            return gate;
        }

        arm_cancel_requested.store(false, std::memory_order_release);

        std::thread([this, task]() {
            int result = 0;

            try {
                {
                    std::lock_guard<std::mutex> robot_lock(robot_io_mutex);
                    result = trackJ2(
                        robot_,
                        task.command_port,
                        task.stream_hz,
                        task.stiffness,
                        [this]() {
                            return this->getTrackJ2FilterParams();
                        },
                        [this](const franka::RobotState& state) {
                            this->updateRobotState(state);
                        },
                        [this]() {
                            return this->isArmCancelRequested();
                        });
                }

                if (result == 0) {
                    arm_status.state.store(arm_state::IDLE);
                } else {
                    arm_status.state.store(arm_state::ERROR);
                }
            } catch (const std::exception& e) {
                std::cerr << "Error executing trackJ2: " << e.what() << std::endl;
                arm_status.state.store(arm_state::ERROR);
            }
        }).detach();

        return RPC_OK;
    } catch (const std::exception& e) {
        std::cerr << "Failed to start trackJ2: " << e.what() << std::endl;
        arm_status.state.store(arm_state::ERROR);
        return RPC_EXECUTION_FAILED;
    }
}

int Robotmanager::startTrackC(const TrackCTask& task) {
    try {
        if (task.command_port <= 0 || task.command_port > 65535) {
            std::cerr << "Invalid trackC command port." << std::endl;
            return RPC_EXECUTION_FAILED;
        }
        if (!std::isfinite(task.stream_hz) || task.stream_hz <= 0.0) {
            std::cerr << "Invalid trackC stream frequency." << std::endl;
            return RPC_EXECUTION_FAILED;
        }

        int gate = try_enter_arm_motion(arm_status);
        if (gate != RPC_OK) {
            if (gate == RPC_DENIED_MOVING) {
                std::cerr << "Arm is currently moving. Cannot start trackC." << std::endl;
            } else if (gate == RPC_DENIED_ERROR) {
                std::cerr << "Arm is in error state. Cannot start trackC." << std::endl;
            }
            return gate;
        }

        arm_cancel_requested.store(false, std::memory_order_release);

        std::thread([this, task]() {
            int result = 0;

            try {
                {
                    std::lock_guard<std::mutex> robot_lock(robot_io_mutex);
                    result = trackC(
                        robot_,
                        task.command_port,
                        task.stream_hz,
                        task.stiffness,
                        task.nullspace_stiffness,
                        [this](const franka::RobotState& state) {
                            this->updateRobotState(state);
                        },
                        [this]() {
                            return this->isArmCancelRequested();
                        });
                }

                if (result == 0) {
                    arm_status.state.store(arm_state::IDLE);
                } else {
                    arm_status.state.store(arm_state::ERROR);
                }
            } catch (const std::exception& e) {
                std::cerr << "Error executing trackC: " << e.what() << std::endl;
                arm_status.state.store(arm_state::ERROR);
            }
        }).detach();

        return RPC_OK;
    } catch (const std::exception& e) {
        std::cerr << "Failed to start trackC: " << e.what() << std::endl;
        arm_status.state.store(arm_state::ERROR);
        return RPC_EXECUTION_FAILED;
    }
}

int Robotmanager::startTrackC2(const TrackCTask& task) {
    try {
        if (task.command_port <= 0 || task.command_port > 65535) {
            std::cerr << "Invalid trackC2 command port." << std::endl;
            return RPC_EXECUTION_FAILED;
        }
        if (!std::isfinite(task.stream_hz) || task.stream_hz <= 0.0) {
            std::cerr << "Invalid trackC2 stream frequency." << std::endl;
            return RPC_EXECUTION_FAILED;
        }

        int gate = try_enter_arm_motion(arm_status);
        if (gate != RPC_OK) {
            if (gate == RPC_DENIED_MOVING) {
                std::cerr << "Arm is currently moving. Cannot start trackC2." << std::endl;
            } else if (gate == RPC_DENIED_ERROR) {
                std::cerr << "Arm is in error state. Cannot start trackC2." << std::endl;
            }
            return gate;
        }

        arm_cancel_requested.store(false, std::memory_order_release);

        std::thread([this, task]() {
            int result = 0;

            try {
                {
                    std::lock_guard<std::mutex> robot_lock(robot_io_mutex);
                    result = trackC2(
                        robot_,
                        task.command_port,
                        task.stream_hz,
                        task.stiffness,
                        task.nullspace_stiffness,
                        [this]() {
                            return this->getTrackC2FilterParams();
                        },
                        [this](const franka::RobotState& state) {
                            this->updateRobotState(state);
                        },
                        [this]() {
                            return this->isArmCancelRequested();
                        });
                }

                if (result == 0) {
                    arm_status.state.store(arm_state::IDLE);
                } else {
                    arm_status.state.store(arm_state::ERROR);
                }
            } catch (const std::exception& e) {
                std::cerr << "Error executing trackC2: " << e.what() << std::endl;
                arm_status.state.store(arm_state::ERROR);
            }
        }).detach();

        return RPC_OK;
    } catch (const std::exception& e) {
        std::cerr << "Failed to start trackC2: " << e.what() << std::endl;
        arm_status.state.store(arm_state::ERROR);
        return RPC_EXECUTION_FAILED;
    }
}


int Robotmanager::executeMoveCartesian(const MoveCartesianTask& task) {
    try {

        // Keep direct moveCartesian compatible with older code: reject while
        // moving instead of enqueueing or preempting the active command.
        if (arm_status.state.load() == arm_state::MOVING) {
            std::cerr << "Arm is currently moving. Cannot execute direct moveCartesian." << std::endl;
            return RPC_DENIED_MOVING;
        }

        if (arm_status.state.load() == arm_state::ERROR) {
            std::cerr << "Arm is in error state. Cannot execute direct moveCartesian." << std::endl;
            return RPC_DENIED_ERROR;
        }

        arm_status.state.store(arm_state::MOVING);
        arm_cancel_requested.store(false, std::memory_order_release);


        int result = 0;
        {
            std::lock_guard<std::mutex> robot_lock(robot_io_mutex);
            result = moveCartesian(
                robot_,
                task.T_d,
                task.stiffness,
                task.vmax_linear,
                task.acc_max_linear,
                task.vmax_angular,
                task.acc_max_angular,
                task.nullspace_stiffness,
                [this](const franka::RobotState& state) {
                    this->updateRobotState(state);
                },
                [this]() {
                    return this->getMotionSlowdownFactor();
                },
                [this]() {
                    return this->isArmCancelRequested();
                }
            );
        }


        if (result == 0) {
            arm_status.state.store(arm_state::IDLE);
            return RPC_OK;
        } else {
            arm_status.state.store(arm_state::ERROR);
            return RPC_EXECUTION_FAILED;
        }

        return result;
    } catch (const std::exception& e) {
        std::cerr << "Error executing direct moveCartesian: " << e.what() << std::endl;
        arm_status.state.store(arm_state::ERROR);
        return RPC_EXECUTION_FAILED;
    }
}

void Robotmanager::enqueueMoveCartesianPath(const MoveCartesianPathTask& task) {
    {
        std::lock_guard<std::mutex> lock(queue_arm_mutex);
        ArmTask arm_task{};
        arm_task.type = ArmCommandType::MOVECART_PATH;
        arm_task.movecart_path_task = task;
        arm_queue.push(arm_task);
    }
    queue_arm_cv.notify_one();
}


int Robotmanager::executeMoveCartesianPath(const MoveCartesianPathTask& task) {
    try {
        int gate = try_enter_arm_motion(arm_status);
        if (gate != RPC_OK) {
            if (gate == RPC_DENIED_MOVING) {
                std::cerr << "Arm is currently moving. Cannot execute direct moveCartesianPath." << std::endl;
            } else if (gate == RPC_DENIED_ERROR) {
                std::cerr << "Arm is in error state. Cannot execute direct moveCartesianPath." << std::endl;
            }
            return gate;
        }

        arm_cancel_requested.store(false, std::memory_order_release);

        int result = 0;
        {
            std::lock_guard<std::mutex> robot_lock(robot_io_mutex);
            result = moveC_path(
                robot_,
                task.waypoints,
                task.samples_per_segment,
                task.path_mode,
                task.stiffness,
                task.vmax_linear,
                task.acc_max_linear,
                task.vmax_angular,
                task.acc_max_angular,
                task.nullspace_stiffness,
                [this](const franka::RobotState& state) {
                    this->updateRobotState(state);
                },
                [this]() {
                    return this->getMotionSlowdownFactor();
                },
                [this]() {
                    return this->isArmCancelRequested();
                });
        }

        if (result == 0) {
            arm_status.state.store(arm_state::IDLE);
            return RPC_OK;
        }

        arm_status.state.store(arm_state::ERROR);
        return RPC_EXECUTION_FAILED;

    } catch (const std::exception& e) {
        std::cerr << "Error executing direct moveCartesianPath: " << e.what() << std::endl;
        arm_status.state.store(arm_state::ERROR);
        return RPC_EXECUTION_FAILED;
    }
}


void Robotmanager::worker_arm_thread() {
    while (!stop_worker.load()) {
        ArmTask task;
        {
            std::unique_lock<std::mutex> lock(queue_arm_mutex);
            // The arm queue is single-consumer. Commands execute serially so a
            // queued path cannot overlap another active arm controller.
            queue_arm_cv.wait(lock, [this]() {
                return !arm_queue.empty() || stop_worker.load();
            });

            if (stop_worker.load() && arm_queue.empty()) {
                return;
            }

            task = arm_queue.front();
            arm_queue.pop();
        }

        switch (task.type) {
            case ArmCommandType::MOVEJ:
                executeMoveJ(task.movej_task);
                break;
            case ArmCommandType::MOVECART:
                executeMoveCartesian(task.movecart_task);
                break;
            case ArmCommandType::MOVEJ_PATH:
                executeMoveJPath(task.movej_path_task);
                break;
            case ArmCommandType::MOVECART_PATH:
                executeMoveCartesianPath(task.movecart_path_task);
                break;
        }
    }
}


int Robotmanager::executeGripperTask(const GripperTask& task) {
    try {
        if (gripper_status.state.load() == gripper_state::MOVING) {
            std::cerr << "Gripper is currently moving. Cannot execute direct gripper task." << std::endl;
            return RPC_DENIED_MOVING;
        }

        if (gripper_status.state.load() == gripper_state::ERROR) {
            std::cerr << "Gripper is in error state. Cannot execute direct gripper task." << std::endl;
            return RPC_DENIED_ERROR;
        }

        gripper_status.state.store(gripper_state::MOVING);
        gripper_status.grasp_success.store(false);

        switch (task.command) {
            case GripperCommand::GRASP: {
                GraspResult result = graspO(
                    gripper_,
                    task.width,
                    task.speed,
                    task.force,
                    task.epsilon_inner,
                    task.epsilon_outer);

                gripper_status.current_width.store(result.current_width);
                gripper_status.target_width.store(task.width);

                if (result.width_too_large) {
                    gripper_status.state.store(gripper_state::WIDTH_TOO_LARGE);
                    gripper_status.grasp_success.store(false);
                    return RPC_EXECUTION_FAILED;
                } else if (result.success && result.is_grasped) {
                    gripper_status.state.store(gripper_state::HOLDING);
                    gripper_status.grasp_success.store(true);
                    return RPC_OK;
                } else {
                    gripper_status.state.store(gripper_state::OPEN_FAILED);
                    gripper_status.grasp_success.store(false);
                    return RPC_EXECUTION_FAILED;
                }
            }

            case GripperCommand::RELEASE: {
                ReleaseResult result = gripperRelease(gripper_, task.speed);

                gripper_status.current_width.store(result.final_width);
                gripper_status.target_width.store(result.final_width);
                gripper_status.grasp_success.store(false);

                if (result.success) {
                    gripper_status.state.store(gripper_state::IDLE);
                    return RPC_OK;
                } else {
                    gripper_status.state.store(gripper_state::ERROR);
                    return RPC_EXECUTION_FAILED;
                }
            }

            case GripperCommand::HOME: {
                int result = gripperHome(gripper_);

                franka::GripperState state = gripper_.readOnce();
                gripper_status.current_width.store(state.width);
                gripper_status.target_width.store(state.width);
                gripper_status.grasp_success.store(false);

                if (result == 0) {
                    gripper_status.state.store(gripper_state::IDLE);
                    return RPC_OK;
                } else {
                    gripper_status.state.store(gripper_state::ERROR);
                    return RPC_EXECUTION_FAILED;
                }
            }
        }

        gripper_status.state.store(gripper_state::ERROR);
        return RPC_EXECUTION_FAILED;
    } catch (const std::exception& e) {
        std::cerr << "Error executing direct gripper task: " << e.what() << std::endl;
        gripper_status.state.store(gripper_state::ERROR);
        gripper_status.grasp_success.store(false);
        return RPC_EXECUTION_FAILED;
    }
}


void Robotmanager::worker_gripper_thread() {
    while (!stop_worker.load()) {
        GripperTask task;
        {
            std::unique_lock<std::mutex> lock(queue_gripper_mutex);
            queue_gripper_cv.wait(lock, [this]() {
                return !gripper_queue.empty() || stop_worker.load();
            });

            if (stop_worker.load() && gripper_queue.empty()) {
                return;
            }

            task = gripper_queue.front();
            gripper_queue.pop();
        }

        executeGripperTask(task);
    }
}
void Robotmanager::signalUdpEvent(){
    udp_event_version_.fetch_add(1,std::memory_order_release);
    udp_event_cv_.notify_all();
}

bool Robotmanager::getRobotStateSnapshot(robot_state_udp& state,std::uint64_t& version){
    std::lock_guard<std::mutex> lock(robot_state_mutex);
    if(!has_robot_state_){
        return false;
    }
    state = latest_robot_state;
    version = robot_state_version_;
    return true;
}

void Robotmanager::updateRobotState(const franka::RobotState& state) {
    robot_state_udp next_state{};

    for (size_t i = 0; i < 7; ++i) {
        next_state.q[i] = state.q[i];
        next_state.dq[i] = state.dq[i];
    }

    for (size_t i = 0; i < 6; ++i) {
        next_state.K_F_ext_hat[i] = state.K_F_ext_hat_K[i];
    }
    // add tau_ext to the state
    for (size_t i = 0; i < 7; ++i) {
        next_state.tau_ext[i] = -state.tau_ext_hat_filtered[i];
    }

    next_state.arm_state = static_cast<int32_t>(arm_status.state.load());
    next_state.gripper_state = static_cast<int32_t>(gripper_status.state.load());

    bool changed = false;
    {
        std::lock_guard<std::mutex> lock(robot_state_mutex);
        // Publish only meaningful state changes. Idle polling still updates the
        // cached state, but unchanged samples do not wake the UDP thread.
        changed = !has_robot_state_ || robotStateChanged(latest_robot_state, next_state);//only if there is no state,or the state truly changed
        latest_robot_state = next_state;

        if (!has_robot_state_ || changed) {
            has_robot_state_ = true;//give the signal that we have the state, and the udp thread can start to publish
            ++robot_state_version_;
        }
    }

    if (changed) {
        signalUdpEvent();
    }
}

robot_state_udp Robotmanager::getLatestRobotState() {
    std::lock_guard<std::mutex> lock(robot_state_mutex);
    return latest_robot_state;
}

void Robotmanager::setUdpTarget(const std::string& ip, int port) {
    std::lock_guard<std::mutex> lock(udp_target_mutex);
    udp_ip_ = ip;
    udp_port_ = port;
    udp_target_set_ = true;
    signalUdpEvent();
}

bool Robotmanager::getUdpTarget(std::string& ip, int& port) {
    std::lock_guard<std::mutex> lock(udp_target_mutex);
    if (!udp_target_set_) {
        return false;
    }

    ip = udp_ip_;
    port = udp_port_;
    return true;
}


void Robotmanager::setUdpFrequency(int frequency_hz) {
    if (frequency_hz <= 0) {
        throw std::runtime_error("UDP frequency must be positive");
    }

    {
        std::lock_guard<std::mutex> lock(udp_target_mutex);
        udp_frequency_hz_ = frequency_hz;
    }
    signalUdpEvent();
}

void Robotmanager::setCollisionBehavior(
    const std::array<double, 7>& lower_torque,
    const std::array<double, 7>& upper_torque,
    const std::array<double, 6>& lower_force,
    const std::array<double, 6>& upper_force) {
    std::lock_guard<std::mutex> robot_lock(robot_io_mutex);
    robot_.setCollisionBehavior(
        // lower_torque,
        // upper_torque,
        // lower_force,
        // upper_force
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}
        // {{200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0}}, {{200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0}},
        // {{200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0}}, {{200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0}},
        // {{200.0, 200.0, 200.0, 200.0, 200.0, 200.0}}, {{200.0, 200.0, 200.0, 200.0, 200.0, 200.0}},
        // {{200.0, 200.0, 200.0, 200.0, 200.0, 200.0}}, {{200.0, 200.0, 200.0, 200.0, 200.0, 200.0}}
        );
}


void Robotmanager::initSession(
    const std::string& udp_ip,
    int udp_port,
    int udp_frequency_hz,
    const std::array<double, 7>& lower_torque,
    const std::array<double, 7>& upper_torque,
    const std::array<double, 6>& lower_force,
    const std::array<double, 6>& upper_force) {

    setUdpTarget(udp_ip, udp_port);
    setUdpFrequency(udp_frequency_hz);
    setCollisionBehavior(
        lower_torque,
        upper_torque,
        lower_force,
        upper_force
    );
    franka::RobotState state;
    {
        std::lock_guard<std::mutex> robot_lock(robot_io_mutex);
        state = robot_.readOnce();
    }
    updateRobotState(state);

    std::cout << "[initSession] UDP target set to " << udp_ip
          << ":" << udp_port
          << ", frequency = " << udp_frequency_hz << " Hz"
          << std::endl;

}

void Robotmanager::worker_udp_thread() {
    using Clock = std::chrono::steady_clock;

    std::string current_ip;
    int current_port = 0;
    std::unique_ptr<UdpStatePublisher> publisher;

    bool warned_not_initialized = false;
    bool has_sent_packet = false;

    auto next_send_allowed = Clock::time_point::min();


    while (!stop_worker.load()) {
        std::string ip;
        int port;

        if (!getUdpTarget(ip, port)) {
            if (!warned_not_initialized) {
                std::cout << "[UDP] Waiting for initSession() to set UDP target." << std::endl;
                warned_not_initialized = true;
            }

            const auto event_version = udp_event_version_.load(std::memory_order_relaxed);
            std::unique_lock<std::mutex> wait_lock(udp_event_mutex_);
            udp_event_cv_.wait_for(
                wait_lock,
                std::chrono::milliseconds(100),
                [this, event_version]() {
                    return stop_worker.load() ||
                           udp_event_version_.load(std::memory_order_relaxed) != event_version;
                });
            continue;
        }

        warned_not_initialized = false;

        if (!publisher || ip != current_ip || port != current_port) {
            publisher.reset(new UdpStatePublisher(ip, port));
            current_ip = ip;
            current_port = port;
            has_sent_packet = false;
            next_send_allowed = Clock::time_point::min();
        }



        robot_state_udp snapshot{};
        std::uint64_t snapshot_version = 0;
        const bool have_state = getRobotStateSnapshot(snapshot, snapshot_version);

        const auto min_send_period = hzToPeriod(getUdpFrequency());
        const auto now = Clock::now();

        const bool can_send_now = !has_sent_packet || now >= next_send_allowed;

        if (have_state && can_send_now) {
            // UDP is best-effort state streaming. The requested frequency is a
            // target; robot control and OS scheduling may make measured Hz lower.
            publisher->publish(snapshot);

            const auto sent_at = Clock::now();
            has_sent_packet = true;
            next_send_allowed = sent_at + min_send_period;
            continue;
        }

        auto wake_deadline = now + std::chrono::milliseconds(100);


        if (has_sent_packet) {
            wake_deadline = std::min(wake_deadline, next_send_allowed);
        }

        const auto event_version = udp_event_version_.load(std::memory_order_relaxed);
        std::unique_lock<std::mutex> wait_lock(udp_event_mutex_);
        udp_event_cv_.wait_until(
            wait_lock,
            wake_deadline,
            [this, event_version]() {
                return stop_worker.load() ||
                       udp_event_version_.load(std::memory_order_relaxed) != event_version;
            });
    }
}
void Robotmanager::setIdleStatePollFrequency(int frequency_hz) {
    if (frequency_hz <= 0) {
        throw std::runtime_error("Idle state poll frequency must be positive");
    }

    udp_idle_poll_hz_.store(frequency_hz, std::memory_order_relaxed);
}

int Robotmanager::getIdleStatePollFrequency() const {
    return udp_idle_poll_hz_.load(std::memory_order_relaxed);
}

int Robotmanager::setMotionSlowdownFactor(double slowdown_factor) {
    if (!std::isfinite(slowdown_factor) || slowdown_factor < 1.0) {
        throw std::runtime_error("Motion slowdown factor must be greater than or equal to 1.0");
    }

    motion_slowdown_factor_.store(slowdown_factor, std::memory_order_release);
    return RPC_OK;
}

double Robotmanager::getMotionSlowdownFactor() const {
    return motion_slowdown_factor_.load(std::memory_order_acquire);
}

int Robotmanager::setTrackJ2FilterParams(double omega_n, double zeta) {
    if (!std::isfinite(omega_n) || omega_n <= 0.0) {
        throw std::runtime_error("TrackJ2 omega_n must be positive");
    }
    if (!std::isfinite(zeta) || zeta <= 0.0) {
        throw std::runtime_error("TrackJ2 zeta must be positive");
    }

    trackj2_omega_n_.store(omega_n, std::memory_order_release);
    trackj2_zeta_.store(zeta, std::memory_order_release);
    return RPC_OK;
}

int Robotmanager::setTrackC2FilterParams(double omega_n, double zeta) {
    if (!std::isfinite(omega_n) || omega_n <= 0.0) {
        throw std::runtime_error("TrackC2 omega_n must be positive");
    }
    if (!std::isfinite(zeta) || zeta <= 0.0) {
        throw std::runtime_error("TrackC2 zeta must be positive");
    }

    trackc2_omega_n_.store(omega_n, std::memory_order_release);
    trackc2_zeta_.store(zeta, std::memory_order_release);
    return RPC_OK;
}

std::pair<double, double> Robotmanager::getTrackJ2FilterParams() const {
    return std::make_pair(
        trackj2_omega_n_.load(std::memory_order_acquire),
        trackj2_zeta_.load(std::memory_order_acquire));
}

std::pair<double, double> Robotmanager::getTrackC2FilterParams() const {
    return std::make_pair(
        trackc2_omega_n_.load(std::memory_order_acquire),
        trackc2_zeta_.load(std::memory_order_acquire));
}

void Robotmanager::worker_robot_state_poll_thread() {
    bool read_error_active = false;
    std::string last_read_error;
    std::size_t suppressed_read_errors = 0;

    while (!stop_worker.load()) {
        const auto poll_period = hzToPeriod(getIdleStatePollFrequency());

        if (arm_status.state.load() != arm_state::MOVING) {
            try {
                franka::RobotState polled_state;

                {
                    std::unique_lock<std::mutex> robot_lock(
                        robot_io_mutex,
                        std::try_to_lock
                    );

                    if (!robot_lock.owns_lock()) {
                        // Do not block an active robot command just to refresh
                        // idle UDP state.
                        std::this_thread::sleep_for(poll_period);
                        continue;
                    }

                    polled_state = robot_.readOnce();
                }

                updateRobotState(polled_state);

                if (read_error_active) {
                    std::cerr << "[state poll] readOnce recovered";
                    if (suppressed_read_errors > 0) {
                        std::cerr << " after suppressing "
                                  << suppressed_read_errors
                                  << " repeated errors";
                    }
                    std::cerr << "." << std::endl;

                    read_error_active = false;
                    last_read_error.clear();
                    suppressed_read_errors = 0;
                }

            } catch (const std::exception& e) {
                const std::string error_message = e.what();

                if (!read_error_active || error_message != last_read_error) {
                    // Log the first repeated readOnce failure, then suppress
                    // identical messages until recovery or a new error appears.
                    std::cerr << "[state poll] readOnce failed: "
                              << error_message
                              << std::endl;

                    read_error_active = true;
                    last_read_error = error_message;
                    suppressed_read_errors = 0;
                } else {
                    ++suppressed_read_errors;
                }
            }
        }

        std::this_thread::sleep_for(poll_period);
    }
}




void Robotmanager::clearMoveQueues() {
    std::lock_guard<std::mutex> lock(queue_arm_mutex);
    // stop/recover clears pending arm work; it does not undo a command that has
    // already entered libfranka control.
    while (!arm_queue.empty()) {
        arm_queue.pop();
    }
}


void Robotmanager::clearGripperQueue() {
    std::lock_guard<std::mutex> lock(queue_gripper_mutex);
    while (!gripper_queue.empty()) {
        gripper_queue.pop();
    }
}

int Robotmanager::recoverArm() {
    try {
        if (arm_status.state.load() == arm_state::MOVING) {
            std::cerr << "Arm is moving. Refuse to recover arm." << std::endl;
            return RPC_DENIED_MOVING;
        }

        clearMoveQueues();

        if (arm_status.state.load() == arm_state::ERROR) {
            std::lock_guard<std::mutex> robot_lock(robot_io_mutex);
            robot_.automaticErrorRecovery();
        }

        franka::RobotState state;
        {
            std::lock_guard<std::mutex> robot_lock(robot_io_mutex);
            state = robot_.readOnce();
        }
        updateRobotState(state);


        arm_status.state.store(arm_state::IDLE);
        return RPC_OK;
    } catch (const std::exception& e) {
        std::cerr << "recoverArm failed: " << e.what() << std::endl;
        arm_status.state.store(arm_state::ERROR);
        return RPC_EXECUTION_FAILED;
    }
}

int Robotmanager::recoverGripper() {
    try {
        if (gripper_status.state.load() == gripper_state::MOVING) {
            std::cerr << "Gripper is moving. Refuse to recover gripper." << std::endl;
            return RPC_DENIED_MOVING;
        }

        clearGripperQueue();

        franka::GripperState state = gripper_.readOnce();
        gripper_status.current_width.store(state.width);
        gripper_status.target_width.store(state.width);
        gripper_status.grasp_success.store(false);
        gripper_status.state.store(gripper_state::IDLE);

        return RPC_OK;
    } catch (const std::exception& e) {
        std::cerr << "recoverGripper failed: " << e.what() << std::endl;
        gripper_status.state.store(gripper_state::ERROR);
        gripper_status.grasp_success.store(false);
        return RPC_EXECUTION_FAILED;
    }
}

int Robotmanager::recoverSystem() {
    if (arm_status.state.load() == arm_state::MOVING ||
        gripper_status.state.load() == gripper_state::MOVING) {
        std::cerr << "System is moving. Refuse to recover system." << std::endl;
        return RPC_DENIED_MOVING;
    }

    int arm_result = recoverArm();
    if (arm_result != RPC_OK) {
        return arm_result;
    }

    int gripper_result = recoverGripper();
    if (gripper_result != RPC_OK) {
        return gripper_result;
    }

    return RPC_OK;
}



int Robotmanager::stopArmMotion() {
    clearMoveQueues();
    arm_cancel_requested.store(true, std::memory_order_release);
    queue_arm_cv.notify_one();
    return RPC_OK;
}

bool Robotmanager::isArmCancelRequested() const {
    return arm_cancel_requested.load(std::memory_order_acquire);
}



int Robotmanager::getUdpFrequency() {
    std::lock_guard<std::mutex> lock(udp_target_mutex);
    return udp_frequency_hz_;
}

franka::RobotState Robotmanager::readRobotState() {
    std::lock_guard<std::mutex> robot_lock(robot_io_mutex);
    return robot_.readOnce();
}

franka::GripperState Robotmanager::readGripperState() {
    return gripper_.readOnce();
}
