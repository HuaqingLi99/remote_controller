#pragma once

#include <xmlrpc-c/base.hpp>
#include <xmlrpc-c/registry.hpp>

class Robotmanager;


/**
 * XML-RPC method adapters exposed by the C++ server.
 *
 * Queue variants return an accept code after pushing a task into Robotmanager.
 * No-queue variants execute immediately and return the execution result code.
 * Units follow libfranka conventions: joints in rad, Cartesian translation in
 * meters, velocities in rad/s or m/s, accelerations in rad/s^2 or m/s^2.
 */

/**
 * RPC "moveJ_queue".
 * Inputs: q_d[7], stiffness[7][7], vmax[7], acc_max[7].
 * Returns: RPC_OK when accepted into the arm queue.
 */
class moveJ_method : public xmlrpc_c::method {
public:
    explicit moveJ_method(Robotmanager& manager);
    void execute(xmlrpc_c::paramList const& paramList,
                 xmlrpc_c::value* const resultp) override;

private:
    Robotmanager& manager_;
};


/**
 * RPC "moveJ".
 * Inputs: q_d[7], stiffness[7][7], vmax[7], acc_max[7].
 * Returns: direct execution result code.
 */
class moveJ_method_no_queue: public xmlrpc_c::method {
public:
    explicit moveJ_method_no_queue(Robotmanager& manager);  
    void execute(xmlrpc_c::paramList const& paramList,
                 xmlrpc_c::value* const resultp) override;
    private:
    Robotmanager& manager_;
};

/**
 * RPC "moveJPath_queue".
 * Inputs: waypoints[N][7], samples_per_segment, path_mode, stiffness[7][7],
 * vmax[7], acc_max[7]. acc_max is kept for API compatibility in replay mode.
 * Returns: RPC_OK when accepted into the arm queue.
 */
class moveJ_path_method : public xmlrpc_c::method {
public:
    explicit moveJ_path_method(Robotmanager& manager);
    void execute(xmlrpc_c::paramList const& paramList,
                 xmlrpc_c::value* const resultp) override;

private:
    Robotmanager& manager_;
};

/**
 * RPC "moveJPath".
 * Same inputs as moveJPath_queue.
 * Returns: direct execution result code.
 */
class moveJ_path_no_queue_method : public xmlrpc_c::method {
public:
    explicit moveJ_path_no_queue_method(Robotmanager& manager);
    void execute(xmlrpc_c::paramList const& paramList,
                 xmlrpc_c::value* const resultp) override;

private:
    Robotmanager& manager_;
};


/**
 * RPC "graspO_queue".
 * Inputs: width[m], speed[m/s], force[N], epsilon_inner[m],
 * epsilon_outer[m].
 * Returns: RPC_OK when accepted into the gripper queue.
 */
class grasp_method : public xmlrpc_c::method {
public:
    explicit grasp_method(Robotmanager& manager);
    void execute(xmlrpc_c::paramList const& paramList,
                 xmlrpc_c::value* const resultp) override;

private:
    Robotmanager& manager_;
};

/**
 * RPC "getArmState".
 * Inputs: none.
 * Returns: integer arm_state enum value.
 */
class get_arm_state_method : public xmlrpc_c::method{
public:
    explicit get_arm_state_method(Robotmanager& manager);
    void execute(xmlrpc_c::paramList const& paramList,
                 xmlrpc_c::value* const resultp) override;

private:
    Robotmanager& manager_;
};

/**
 * RPC "getGripperState".
 * Inputs: none.
 * Returns: integer gripper_state enum value.
 */
class get_gripper_state_method : public xmlrpc_c::method {
public:
    explicit get_gripper_state_method(Robotmanager& manager);
    void execute(xmlrpc_c::paramList const& paramList,
                 xmlrpc_c::value* const resultp) override;

private:
    Robotmanager& manager_;
};

/**
 * RPC "gripperHome_queue".
 * Inputs: none.
 * Returns: RPC_OK when accepted into the gripper queue.
 */
class gripper_home_method : public xmlrpc_c::method {
public:
    explicit gripper_home_method(Robotmanager& manager);
    void execute(xmlrpc_c::paramList const& paramList,
                 xmlrpc_c::value* const resultp) override;

private:
    Robotmanager& manager_;
};

/**
 * RPC "gripperRelease_queue".
 * Inputs: speed[m/s].
 * Returns: RPC_OK when accepted into the gripper queue.
 */
class gripper_release_method : public xmlrpc_c::method {
public:
    explicit gripper_release_method(Robotmanager& manager);
    void execute(xmlrpc_c::paramList const& paramList,
                 xmlrpc_c::value* const resultp) override;

private:
    Robotmanager& manager_;
};

/**
 * RPC "graspO".
 * Inputs: width[m], speed[m/s], force[N], epsilon_inner[m],
 * epsilon_outer[m].
 * Returns: direct execution result code.
 */
class grasp_no_queue_method : public xmlrpc_c::method {
public:
    explicit grasp_no_queue_method(Robotmanager& manager);
    void execute(xmlrpc_c::paramList const& paramList,
                 xmlrpc_c::value* const resultp) override;

private:
    Robotmanager& manager_;
};

/**
 * RPC "gripperHome".
 * Inputs: none.
 * Returns: direct execution result code.
 */
class gripper_home_no_queue_method : public xmlrpc_c::method {
public:
    explicit gripper_home_no_queue_method(Robotmanager& manager);
    void execute(xmlrpc_c::paramList const& paramList,
                 xmlrpc_c::value* const resultp) override;

private:
    Robotmanager& manager_;
};

/**
 * RPC "gripperRelease".
 * Inputs: speed[m/s].
 * Returns: direct execution result code.
 */
class gripper_release_no_queue_method : public xmlrpc_c::method {
public:
    explicit gripper_release_no_queue_method(Robotmanager& manager);
    void execute(xmlrpc_c::paramList const& paramList,
                 xmlrpc_c::value* const resultp) override;

private:
    Robotmanager& manager_;
};




/**
 * RPC "initSession".
 * Inputs: UDP target ip, UDP target port, UDP frequency[Hz],
 * lower/upper torque thresholds[7], lower/upper force thresholds[6].
 * Returns: RPC_OK after configuring session state.
 */
class init_session_method : public xmlrpc_c::method {
public:
    explicit init_session_method(Robotmanager& manager);
    void execute(xmlrpc_c::paramList const& paramList,
                 xmlrpc_c::value* const resultp) override;

private:
    Robotmanager& manager_;
};

/**
 * RPC "moveCartesian_queue".
 * Inputs: T_d[16] row-major pose in base/world frame, stiffness[6][6],
 * vmax_linear[m/s], acc_max_linear[m/s^2], vmax_angular[rad/s],
 * acc_max_angular[rad/s^2], nullspace_stiffness.
 * Returns: RPC_OK when accepted into the arm queue.
 */
class move_cartesian_method : public xmlrpc_c::method {
public:
    explicit move_cartesian_method(Robotmanager& manager);
    void execute(xmlrpc_c::paramList const& paramList,
                 xmlrpc_c::value* const resultp) override;

private:
    Robotmanager& manager_;
};

/**
 * RPC "moveCartesian".
 * Same inputs as moveCartesian_queue.
 * Returns: direct execution result code.
 */
class move_cartesian_no_queue_method : public xmlrpc_c::method {
public:
    explicit move_cartesian_no_queue_method(Robotmanager& manager);
    void execute(xmlrpc_c::paramList const& paramList,
                 xmlrpc_c::value* const resultp) override;

private:
    Robotmanager& manager_;
};


/**
 * RPC "recoverSystem".
 * Inputs: none.
 * Returns: recovery result code.
 */
class recover_system_method : public xmlrpc_c::method {
public:
    explicit recover_system_method(Robotmanager& manager);
    void execute(xmlrpc_c::paramList const& paramList,
                 xmlrpc_c::value* const resultp) override;

private:
    Robotmanager& manager_;
};

/**
 * RPC "setIdleStatePollFrequency".
 * Inputs: frequency_hz.
 * Returns: RPC_OK after updating the idle polling frequency.
 */
class set_idle_state_poll_frequency_method : public xmlrpc_c::method {
public:
    explicit set_idle_state_poll_frequency_method(Robotmanager& manager);
    void execute(xmlrpc_c::paramList const& paramList,
                 xmlrpc_c::value* const resultp) override;

private:
    Robotmanager& manager_;
};

/**
 * RPC "setSlowdownFactor".
 * Inputs: slowdown_factor >= 1.0. Can be called while an arm motion is moving.
 * Returns: RPC_OK after updating the runtime motion slowdown.
 */
class set_slowdown_factor_method : public xmlrpc_c::method {
public:
    explicit set_slowdown_factor_method(Robotmanager& manager);
    void execute(xmlrpc_c::paramList const& paramList,
                 xmlrpc_c::value* const resultp) override;

private:
    Robotmanager& manager_;
};

/**
 * RPC "getSlowdownFactor".
 * Inputs: none.
 * Returns: current runtime motion slowdown.
 */
class get_slowdown_factor_method : public xmlrpc_c::method {
public:
    explicit get_slowdown_factor_method(Robotmanager& manager);
    void execute(xmlrpc_c::paramList const& paramList,
                 xmlrpc_c::value* const resultp) override;

private:
    Robotmanager& manager_;
};

/**
 * RPC "startTrackJ".
 * Inputs: command_port, stream_hz, stiffness[7][7].
 * Returns: result code after starting the streaming joint controller.
 */
class start_trackj_method : public xmlrpc_c::method {
public:
    explicit start_trackj_method(Robotmanager& manager);
    void execute(xmlrpc_c::paramList const& paramList,
                 xmlrpc_c::value* const resultp) override;

private:
    Robotmanager& manager_;
};

/**
 * RPC "startTrackJ2".
 * Inputs: command_port, stream_hz, stiffness[7][7].
 * Returns: result code after starting the async target joint controller.
 */
class start_trackj2_method : public xmlrpc_c::method {
public:
    explicit start_trackj2_method(Robotmanager& manager);
    void execute(xmlrpc_c::paramList const& paramList,
                 xmlrpc_c::value* const resultp) override;

private:
    Robotmanager& manager_;
};

/**
 * RPC "startTrackC".
 * Inputs: command_port, stream_hz, stiffness[6][6], nullspace_stiffness.
 * Returns: result code after starting the streaming Cartesian controller.
 */
class start_trackc_method : public xmlrpc_c::method {
public:
    explicit start_trackc_method(Robotmanager& manager);
    void execute(xmlrpc_c::paramList const& paramList,
                 xmlrpc_c::value* const resultp) override;

private:
    Robotmanager& manager_;
};

/**
 * RPC "startTrackC2".
 * Inputs: command_port, stream_hz, stiffness[6][6], nullspace_stiffness.
 * Returns: result code after starting the async target Cartesian controller.
 */
class start_trackc2_method : public xmlrpc_c::method {
public:
    explicit start_trackc2_method(Robotmanager& manager);
    void execute(xmlrpc_c::paramList const& paramList,
                 xmlrpc_c::value* const resultp) override;

private:
    Robotmanager& manager_;
};

/**
 * RPC "setTrackJ2FilterParams".
 * Inputs: omega_n, zeta.
 * Returns: result code.
 */
class set_trackj2_filter_params_method : public xmlrpc_c::method {
public:
    explicit set_trackj2_filter_params_method(Robotmanager& manager);
    void execute(xmlrpc_c::paramList const& paramList,
                 xmlrpc_c::value* const resultp) override;

private:
    Robotmanager& manager_;
};

/**
 * RPC "setTrackC2FilterParams".
 * Inputs: omega_n, zeta.
 * Returns: result code.
 */
class set_trackc2_filter_params_method : public xmlrpc_c::method {
public:
    explicit set_trackc2_filter_params_method(Robotmanager& manager);
    void execute(xmlrpc_c::paramList const& paramList,
                 xmlrpc_c::value* const resultp) override;

private:
    Robotmanager& manager_;
};

/**
 * RPC "stopArmMotion".
 * Inputs: none.
 * Returns: result code after requesting stop and clearing queued arm tasks.
 */
class stop_arm_motion_method : public xmlrpc_c::method {
public:
    explicit stop_arm_motion_method(Robotmanager& manager);
    void execute(xmlrpc_c::paramList const& paramList,
                 xmlrpc_c::value* const resultp) override;

private:
    Robotmanager& manager_;
};

/**
 * RPC "moveCartesianPath_queue".
 * Inputs: waypoints[N][16] row-major poses, samples_per_segment, path_mode,
 * stiffness[6][6], vmax_linear[m/s], acc_max_linear accepted
 * but not used, vmax_angular[rad/s], acc_max_angular accepted but not used,
 * nullspace_stiffness.
 * Returns: RPC_OK when accepted into the arm queue.
 */
class move_cartesian_path_method : public xmlrpc_c::method {
public:
    explicit move_cartesian_path_method(Robotmanager& manager);
    void execute(xmlrpc_c::paramList const& paramList,
                 xmlrpc_c::value* const resultp) override;

private:
    Robotmanager& manager_;
};

/**
 * RPC "moveCartesianPath".
 * Same inputs as moveCartesianPath_queue.
 * Returns: direct execution result code.
 */
class move_cartesian_path_no_queue_method : public xmlrpc_c::method {
public:
    explicit move_cartesian_path_no_queue_method(Robotmanager& manager);
    void execute(xmlrpc_c::paramList const& paramList,
                 xmlrpc_c::value* const resultp) override;

private:
    Robotmanager& manager_;
};

/**
 * RPC "getGripperWidth".
 * Inputs: none.
 * Returns: cached gripper width in meters.
 */
class get_gripper_width_method : public xmlrpc_c::method {
public:
    explicit get_gripper_width_method(Robotmanager& manager);

    void execute(
        xmlrpc_c::paramList const& paramList,
        xmlrpc_c::value* const resultp
    ) override;

private:
    Robotmanager& manager_;
};


/**
 * Register all XML-RPC methods used by the Python client.
 */
void register_methods(xmlrpc_c::registry& registry, Robotmanager& manager);
