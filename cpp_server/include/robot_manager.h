#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <string>
#include <utility>
#include <vector>
#include "moveJ_path.h"
#include <cstdint>
#include "move_cartesian_path.h"



#include <franka/gripper.h>
#include <franka/robot.h>

#include <udp_state_publisher.h>

constexpr int RPC_OK = 0;
constexpr int RPC_DENIED_MOVING = -2;
constexpr int RPC_DENIED_ERROR = -3;
constexpr int RPC_EXECUTION_FAILED = -4;



/**
 * Arm command lifecycle exposed through getArmState.
 */
enum class arm_state
{
    IDLE,
    MOVING,
    ERROR
};

/**
 * Gripper command lifecycle exposed through getGripperState.
 */
enum class gripper_state
{
    IDLE,
    MOVING,
    ERROR,
    WIDTH_TOO_LARGE,
    HOLDING,
    OPEN_FAILED
};

/**
 * Internal gripper task type used by the gripper worker queue.
 */
enum class GripperCommand
{
    GRASP,
    RELEASE,
    HOME
};

/**
 * Thread-safe arm status shared by RPC methods and worker threads.
 */
struct ArmStatus
{
    std::atomic<arm_state> state{arm_state::IDLE};
};

/**
 * Thread-safe gripper status shared by RPC methods and worker threads.
 *
 * current_width and target_width are meters.
 */
struct GripperStatus
{
    std::atomic<gripper_state> state{gripper_state::IDLE};
    std::atomic<double> current_width{0.0};
    std::atomic<double> target_width{0.0};
    std::atomic<bool> grasp_success{false};
};

/**
 * Single-target joint command.
 *
 * q_d is in rad, vmax is in rad/s, and acc_max is in rad/s^2.
 */
struct MoveJTask
{
    std::array<double, 7> q_d;
    std::array<std::array<double, 7>, 7> stiffness;
    std::array<double, 7> vmax;
    std::array<double, 7> acc_max;
};

/**
 * Multi-waypoint joint command.
 *
 * waypoints are Nx7 joint vectors in rad. acc_max is retained for API
 * compatibility; moveJ_path replay currently uses samples_per_segment and a
 * runtime motion slowdown factor for timing and checks dq limits.
 */
struct MoveJPathTask
{
    std::vector<std::array<double, 7>> waypoints;
    int samples_per_segment = 10;
    Pathmode path_mode = Pathmode::Linear;
    std::array<std::array<double, 7>, 7> stiffness;
    std::array<double, 7> vmax;
    std::array<double, 7> acc_max;
};

/**
 * Streaming joint command.
 *
 * startTrackJ starts a torque controller that tracks q_ref samples received on
 * command_port over UDP. stream_hz is used by the controller watchdog.
 */
struct TrackJTask
{
    int command_port = 0;
    double stream_hz = 0.0;
    std::array<std::array<double, 7>, 7> stiffness;
};

/**
 * Streaming Cartesian command.
 *
 * startTrackC starts a torque controller that tracks row-major 4x4 T_ref
 * samples received on command_port over UDP.
 */
struct TrackCTask
{
    int command_port = 0;
    double stream_hz = 0.0;
    std::array<std::array<double, 6>, 6> stiffness;
    double nullspace_stiffness = 0.0;
};

/**
 * Multi-waypoint Cartesian command.
 *
 * waypoints are Nx16 row-major 4x4 poses in base/world frame. acc_max_* is
 * accepted for API compatibility; moveC_path replay currently uses
 * samples_per_segment and a runtime motion slowdown factor for timing and checks
 * vmax_* limits.
 */
struct MoveCartesianPathTask
{
    std::vector<std::array<double, 16>> waypoints;
    int samples_per_segment = 10;
    Pathmode path_mode = Pathmode::Linear;
    std::array<std::array<double, 6>, 6> stiffness;
    double vmax_linear = 0.0;
    double acc_max_linear = 0.0;
    double vmax_angular = 0.0;
    double acc_max_angular = 0.0;
    double nullspace_stiffness = 0.0;
};


/**
 * Gripper command payload.
 *
 * width and epsilon values are meters, speed is m/s, and force is newtons.
 */
struct GripperTask
{
    GripperCommand command;
    double width = 0.0;
    double speed = 0.0;
    double force = 0.0;
    double epsilon_inner = 0.0;
    double epsilon_outer = 0.0;
};

/**
 * Single-target Cartesian command.
 *
 * T_d is a row-major flattened 4x4 pose in base/world frame.
 */
struct MoveCartesianTask
{
    std::array<double, 16> T_d;
    std::array<std::array<double, 6>, 6> stiffness;
    double vmax_linear;
    double acc_max_linear;
    double vmax_angular;
    double acc_max_angular;
    double nullspace_stiffness;
};

/**
 * Internal arm worker command discriminator.
 */
enum class ArmCommandType
{
    MOVEJ,
    MOVECART,
    MOVEJ_PATH,
    MOVECART_PATH
};

/**
 * Internal arm queue payload. Only the task matching type is consumed.
 */
struct ArmTask
{
    ArmCommandType type;
    MoveJTask movej_task;
    MoveJPathTask movej_path_task;
    MoveCartesianTask movecart_task;
    MoveCartesianPathTask movecart_path_task;

};








/**
 * Runtime coordinator for arm commands, gripper commands, recovery, and UDP.
 *
 * Direct execution methods return an RPC result code. Enqueue methods push work
 * into the corresponding worker queue and return immediately through the RPC
 * adapter.
 */
class Robotmanager {
public:
    explicit Robotmanager(franka::Robot& robot, franka::Gripper& gripper);

    /**
     * Add a joint-space command to the arm queue.
     */
    void enqueueMoveJ(const MoveJTask& task);

    /**
     * Execute one joint-space command immediately when the arm is idle.
     */
    int executeMoveJ(const MoveJTask& task);

    /**
     * Add a joint-space path command to the arm queue.
     */
    void enqueueMoveJPath(const MoveJPathTask& task);

    /**
     * Execute one joint-space path command immediately when the arm is idle.
     */
    int executeMoveJPath(const MoveJPathTask& task);

    /**
     * Start streaming joint tracking and return immediately.
     */
    int startTrackJ(const TrackJTask& task);

    /**
     * Start async target joint tracking with server-side second-order filtering.
     */
    int startTrackJ2(const TrackJTask& task);

    /**
     * Start streaming Cartesian tracking and return immediately.
     */
    int startTrackC(const TrackCTask& task);

    /**
     * Start async target Cartesian tracking with server-side second-order filtering.
     */
    int startTrackC2(const TrackCTask& task);

    /**
     * Set TrackJ2 second-order filter parameters.
     */
    int setTrackJ2FilterParams(double omega_n, double zeta);

    /**
     * Set TrackC2 second-order filter parameters.
     */
    int setTrackC2FilterParams(double omega_n, double zeta);

    /**
     * Get TrackJ2 second-order filter parameters as omega_n, zeta.
     */
    std::pair<double, double> getTrackJ2FilterParams() const;

    /**
     * Get TrackC2 second-order filter parameters as omega_n, zeta.
     */
    std::pair<double, double> getTrackC2FilterParams() const;


    /**
     * Add a gripper command to the gripper queue.
     */
    void enqueueGripper(const GripperTask& task);

    /**
     * Execute one gripper command immediately when the gripper is idle.
     */
    int executeGripperTask(const GripperTask& task);


    /**
     * Add a Cartesian single-pose command to the arm queue.
     */
    void enqueueMoveCartesian(const MoveCartesianTask& task);

    /**
     * Execute one Cartesian single-pose command immediately when the arm is idle.
     */
    int executeMoveCartesian(const MoveCartesianTask& task);

    /**
     * Add a Cartesian path command to the arm queue.
     */
    void enqueueMoveCartesianPath(const MoveCartesianPathTask& task);

    /**
     * Execute one Cartesian path command immediately when the arm is idle.
     */
    int executeMoveCartesianPath(const MoveCartesianPathTask& task);


    /**
     * Set how often idle polling refreshes robot state for UDP feedback.
     */
    void setIdleStatePollFrequency(int frequency_hz);

    /**
     * Return idle robot-state polling frequency in Hz.
     */
    int getIdleStatePollFrequency() const;

    /**
     * Set runtime arm motion replay slowdown. 1.0 is the original speed.
     */
    int setMotionSlowdownFactor(double slowdown_factor);

    /**
     * Return runtime arm motion replay slowdown.
     */
    double getMotionSlowdownFactor() const;



    /**
     * Set UDP publish frequency in Hz.
     */
    void setUdpFrequency(int frequency_hz);

    /**
     * Return configured UDP publish frequency in Hz.
     */
    int getUdpFrequency();


    /**
     * Worker loops for queued gripper, UDP, idle polling, and queued arm work.
     */
    void worker_gripper_thread();
    void worker_udp_thread();
    void worker_robot_state_poll_thread();
    void worker_arm_thread();



    /**
     * Cache the latest robot state from a control callback or idle read.
     */
    void updateRobotState(const franka::RobotState& state);

    /**
     * Return the latest cached UDP packet payload.
     */
    robot_state_udp getLatestRobotState();

    /**
     * Configure the UDP feedback destination.
     */
    void setUdpTarget(const std::string& ip, int port);

    /**
     * Read the configured UDP destination.
     */
    bool getUdpTarget(std::string& ip, int& port);

    /**
     * Apply Franka collision behavior thresholds.
     */
    void setCollisionBehavior(
        const std::array<double, 7>& lower_torque,
        const std::array<double, 7>& upper_torque,
        const std::array<double, 6>& lower_force,
        const std::array<double, 6>& upper_force);

    /**
     * Configure UDP destination/frequency and collision behavior for a session.
     */
    void initSession(
        const std::string& udp_ip,
        int udp_port,
        int udp_frequency_hz,
        const std::array<double, 7>& lower_torque,
        const std::array<double, 7>& upper_torque,
        const std::array<double, 6>& lower_force,
        const std::array<double, 6>& upper_force);

    /**
     * Blocking single hardware state reads guarded by robot_io_mutex.
     */
    franka::RobotState readRobotState();
    franka::GripperState readGripperState();

    /**
     * Recover arm, gripper, or both from an error state.
     */
    int recoverArm();
    int recoverGripper();
    int recoverSystem();

    /**
     * Request current arm motion to stop and clear queued arm tasks.
     */
    int stopArmMotion();

    /**
     * Return true after stopArmMotion has requested cancellation.
     */
    bool isArmCancelRequested() const;


    


    std::queue<ArmTask> arm_queue;
    std::mutex queue_arm_mutex;
    std::condition_variable queue_arm_cv;


    std::queue<GripperTask> gripper_queue;
    std::mutex queue_gripper_mutex;
    std::condition_variable queue_gripper_cv;


    ArmStatus arm_status;
    GripperStatus gripper_status;

    robot_state_udp latest_robot_state{};
    std::mutex robot_state_mutex;

    std::atomic<bool> stop_worker{false};




private:
    void clearMoveQueues();
    void clearGripperQueue();

    void signalUdpEvent();
    bool getRobotStateSnapshot(robot_state_udp& state, std::uint64_t& version);

    franka::Robot& robot_;
    franka::Gripper& gripper_;

    std::mutex robot_io_mutex;
    std::atomic<int> udp_idle_poll_hz_{5};

    std::mutex udp_target_mutex;
    std::string udp_ip_;
    int udp_port_ = 0;
    bool udp_target_set_ = false;
    int udp_frequency_hz_ = 100;
    std::atomic<bool> arm_cancel_requested{false};
    std::atomic<double> motion_slowdown_factor_{1.0};
    std::atomic<double> trackj2_omega_n_{10.0};
    std::atomic<double> trackj2_zeta_{1.0};
    std::atomic<double> trackc2_omega_n_{10.0};
    std::atomic<double> trackc2_zeta_{1.0};



    //only weak when get udp
    std::condition_variable udp_event_cv_;
    std::mutex udp_event_mutex_;
    std::atomic<std::uint64_t> udp_event_version_{0};//++,when new event come

    bool has_robot_state_ = false;
    std::uint64_t robot_state_version_ = 0;



};
