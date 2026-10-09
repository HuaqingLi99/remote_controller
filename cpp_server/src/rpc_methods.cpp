#include "rpc_methods.h"

#include "robot_manager.h"
#include "moveJ_path.h"
#include <array>
#include <stdexcept>
#include <vector>

/**
 * XML-RPC method adapters.
 *
 * This file validates incoming XML-RPC arrays/scalars, converts them into typed
 * task structs, and dispatches those tasks to Robotmanager. Keep the parser
 * conventions in sync with the Python client, especially row-major flattened
 * 4x4 poses and numeric values sent as XML-RPC doubles.
 */

static MoveJTask parse_movej_task(xmlrpc_c::paramList const& paramList) {
    // The current parser expects XML-RPC double values. Python callers should
    // cast numeric arrays to float before sending, especially zeros.
    xmlrpc_c::value_array q_d_array = paramList.getArray(0);
    std::vector<xmlrpc_c::value> elements = q_d_array.vectorValueValue();
    std::vector<double> q_d;
    q_d.reserve(elements.size());
    for (const auto& element : elements) {
        q_d.push_back(static_cast<double>(xmlrpc_c::value_double(element)));
    }

    xmlrpc_c::value_array stiffness_array = paramList.getArray(1);
    std::vector<xmlrpc_c::value> stiffness_elements = stiffness_array.vectorValueValue();
    std::vector<std::vector<double>> stiffness;
    stiffness.reserve(stiffness_elements.size());

    for (const auto& stiffness_row : stiffness_elements) {
        xmlrpc_c::value_array row_array(stiffness_row);
        std::vector<xmlrpc_c::value> row_elements = row_array.vectorValueValue();
        std::vector<double> row;
        row.reserve(row_elements.size());
        for (const auto& element : row_elements) {
            row.push_back(static_cast<double>(xmlrpc_c::value_double(element)));
        }
        stiffness.push_back(row);
    }

    xmlrpc_c::value_array vmax_array = paramList.getArray(2);
    std::vector<xmlrpc_c::value> vmax_elements = vmax_array.vectorValueValue();
    std::vector<double> vmax;
    vmax.reserve(vmax_elements.size());
    for (const auto& element : vmax_elements) {
        vmax.push_back(static_cast<double>(xmlrpc_c::value_double(element)));
    }

    xmlrpc_c::value_array acc_max_array = paramList.getArray(3);
    std::vector<xmlrpc_c::value> acc_max_elements = acc_max_array.vectorValueValue();
    std::vector<double> acc_max;
    acc_max.reserve(acc_max_elements.size());
    for (const auto& element : acc_max_elements) {
        acc_max.push_back(static_cast<double>(xmlrpc_c::value_double(element)));
    }

    paramList.verifyEnd(4);

    if (q_d.size() != 7 || vmax.size() != 7 || acc_max.size() != 7 || stiffness.size() != 7) {
        throw std::runtime_error("Invalid input size");
    }

    for (const auto& row : stiffness) {
        if (row.size() != 7) {
            throw std::runtime_error("Invalid stiffness row size");
        }
    }

    std::array<double, 7> q_d_array_value{};
    for (size_t i = 0; i < 7; ++i) {
        q_d_array_value[i] = q_d[i];
    }

    std::array<std::array<double, 7>, 7> stiffness_array_value{};
    for (size_t i = 0; i < 7; ++i) {
        for (size_t j = 0; j < 7; ++j) {
            stiffness_array_value[i][j] = stiffness[i][j];
        }
    }

    std::array<double, 7> vmax_array_value{};
    for (size_t i = 0; i < 7; ++i) {
        vmax_array_value[i] = vmax[i];
    }

    std::array<double, 7> acc_max_array_value{};
    for (size_t i = 0; i < 7; ++i) {
        acc_max_array_value[i] = acc_max[i];
    }

    return MoveJTask{q_d_array_value, stiffness_array_value, vmax_array_value, acc_max_array_value};
}
static MoveJPathTask parse_movej_path_task(xmlrpc_c::paramList const& paramList) {
    // Waypoints are replayed as fixed-size joint vectors; keep validation here
    // before the task enters the robot manager queue.
    xmlrpc_c::value_array waypoints_array = paramList.getArray(0);
    std::vector<xmlrpc_c::value> waypoint_rows = waypoints_array.vectorValueValue();

    std::vector<std::array<double, 7>> waypoints;
    waypoints.reserve(waypoint_rows.size());

    for (const auto& row_value : waypoint_rows) {
        xmlrpc_c::value_array row_array(row_value);
        std::vector<xmlrpc_c::value> row = row_array.vectorValueValue();
        if (row.size() != 7) {
            throw std::runtime_error("Each waypoint must have size 7");
        }

        std::array<double, 7> waypoint{};
        for (size_t i = 0; i < 7; ++i) {
            waypoint[i] = static_cast<double>(xmlrpc_c::value_double(row[i]));
        }
        waypoints.push_back(waypoint);
    }

    int samples_per_segment = paramList.getInt(1);
    std::string mode_str = paramList.getString(2);

    Pathmode path_mode;
    if (mode_str == "linear") {
        path_mode = Pathmode::Linear;
    } else if (mode_str == "catmull_rom") {
        path_mode = Pathmode::CatmullRomSpline;
    } else {
        throw std::runtime_error("path_mode must be 'linear' or 'catmull_rom'");
    }

    xmlrpc_c::value_array stiffness_array = paramList.getArray(3);
    std::vector<xmlrpc_c::value> stiffness_rows = stiffness_array.vectorValueValue();
    if (stiffness_rows.size() != 7) {
        throw std::runtime_error("stiffness must have 7 rows");
    }

    std::array<std::array<double, 7>, 7> stiffness{};
    for (size_t i = 0; i < 7; ++i) {
        xmlrpc_c::value_array row_array(stiffness_rows[i]);
        std::vector<xmlrpc_c::value> row = row_array.vectorValueValue();
        if (row.size() != 7) {
            throw std::runtime_error("Each stiffness row must have 7 elements");
        }
        for (size_t j = 0; j < 7; ++j) {
            stiffness[i][j] = static_cast<double>(xmlrpc_c::value_double(row[j]));
        }
    }

    xmlrpc_c::value_array vmax_array = paramList.getArray(4);
    std::vector<xmlrpc_c::value> vmax_values = vmax_array.vectorValueValue();
    if (vmax_values.size() != 7) {
        throw std::runtime_error("vmax must have size 7");
    }
    std::array<double, 7> vmax{};
    for (size_t i = 0; i < 7; ++i) {
        vmax[i] = static_cast<double>(xmlrpc_c::value_double(vmax_values[i]));
    }

    xmlrpc_c::value_array acc_array = paramList.getArray(5);
    std::vector<xmlrpc_c::value> acc_values = acc_array.vectorValueValue();
    if (acc_values.size() != 7) {
        throw std::runtime_error("acc_max must have size 7");
    }
    std::array<double, 7> acc_max{};
    for (size_t i = 0; i < 7; ++i) {
        acc_max[i] = static_cast<double>(xmlrpc_c::value_double(acc_values[i]));
    }

    paramList.verifyEnd(6);

    MoveJPathTask task{};
    task.waypoints = std::move(waypoints);
    task.samples_per_segment = samples_per_segment;
    task.path_mode = path_mode;
    task.stiffness = stiffness;
    task.vmax = vmax;
    task.acc_max = acc_max;
    return task;
}

static TrackJTask parse_trackj_task(xmlrpc_c::paramList const& paramList) {
    int command_port = paramList.getInt(0);
    double stream_hz = paramList.getDouble(1);

    xmlrpc_c::value_array stiffness_array = paramList.getArray(2);
    std::vector<xmlrpc_c::value> stiffness_rows = stiffness_array.vectorValueValue();
    if (stiffness_rows.size() != 7) {
        throw std::runtime_error("stiffness must have 7 rows");
    }

    std::array<std::array<double, 7>, 7> stiffness{};
    for (size_t i = 0; i < 7; ++i) {
        xmlrpc_c::value_array row_array(stiffness_rows[i]);
        std::vector<xmlrpc_c::value> row = row_array.vectorValueValue();
        if (row.size() != 7) {
            throw std::runtime_error("Each stiffness row must have 7 elements");
        }

        for (size_t j = 0; j < 7; ++j) {
            stiffness[i][j] = static_cast<double>(xmlrpc_c::value_double(row[j]));
        }
    }

    paramList.verifyEnd(3);

    TrackJTask task{};
    task.command_port = command_port;
    task.stream_hz = stream_hz;
    task.stiffness = stiffness;
    return task;
}

static TrackCTask parse_trackc_task(xmlrpc_c::paramList const& paramList) {
    int command_port = paramList.getInt(0);
    double stream_hz = paramList.getDouble(1);

    xmlrpc_c::value_array stiffness_array = paramList.getArray(2);
    std::vector<xmlrpc_c::value> stiffness_rows = stiffness_array.vectorValueValue();
    if (stiffness_rows.size() != 6) {
        throw std::runtime_error("Cartesian stiffness must have 6 rows");
    }

    std::array<std::array<double, 6>, 6> stiffness{};
    for (size_t i = 0; i < 6; ++i) {
        xmlrpc_c::value_array row_array(stiffness_rows[i]);
        std::vector<xmlrpc_c::value> row = row_array.vectorValueValue();
        if (row.size() != 6) {
            throw std::runtime_error("Each Cartesian stiffness row must have 6 elements");
        }

        for (size_t j = 0; j < 6; ++j) {
            stiffness[i][j] = static_cast<double>(xmlrpc_c::value_double(row[j]));
        }
    }

    double nullspace_stiffness = paramList.getDouble(3);
    paramList.verifyEnd(4);

    TrackCTask task{};
    task.command_port = command_port;
    task.stream_hz = stream_hz;
    task.stiffness = stiffness;
    task.nullspace_stiffness = nullspace_stiffness;
    return task;
}



static MoveCartesianTask parse_move_cartesian_task(xmlrpc_c::paramList const& paramList) {
    // T_d is a row-major flattened 4x4 pose from the Python client.
    // moveC.cpp reconstructs it with the same row-major convention.
    xmlrpc_c::value_array T_array = paramList.getArray(0);
    std::vector<xmlrpc_c::value> T_elements = T_array.vectorValueValue();
    if (T_elements.size() != 16) {
        throw std::runtime_error("T_d must have size 16");
    }

    std::array<double, 16> T_d{};
    for (size_t i = 0; i < 16; ++i) {
        T_d[i] = static_cast<double>(xmlrpc_c::value_double(T_elements[i]));
    }

    xmlrpc_c::value_array stiffness_array = paramList.getArray(1);
    std::vector<xmlrpc_c::value> stiffness_elements = stiffness_array.vectorValueValue();
    if (stiffness_elements.size() != 6) {
        throw std::runtime_error("Cartesian stiffness must have 6 rows");
    }

    std::array<std::array<double, 6>, 6> stiffness{};
    for (size_t i = 0; i < 6; ++i) {
        xmlrpc_c::value_array row_array(stiffness_elements[i]);
        std::vector<xmlrpc_c::value> row_elements = row_array.vectorValueValue();
        if (row_elements.size() != 6) {
            throw std::runtime_error("Each Cartesian stiffness row must have 6 elements");
        }
        for (size_t j = 0; j < 6; ++j) {
            stiffness[i][j] = static_cast<double>(xmlrpc_c::value_double(row_elements[j]));
        }
    }

    double vmax_linear = paramList.getDouble(2);
    double acc_max_linear = paramList.getDouble(3);
    double vmax_angular = paramList.getDouble(4);
    double acc_max_angular = paramList.getDouble(5);
    double nullspace_stiffness = paramList.getDouble(6);

    paramList.verifyEnd(7);
    return MoveCartesianTask{T_d, stiffness, vmax_linear, acc_max_linear, vmax_angular, acc_max_angular, nullspace_stiffness};
}
static MoveCartesianPathTask parse_move_cartesian_path_task(
    xmlrpc_c::paramList const& paramList) {

    // Each Cartesian waypoint is one row-major flattened 4x4 pose.
    // The path controller prepends the robot's current O_T_EE as the start pose.
    xmlrpc_c::value_array waypoints_array = paramList.getArray(0);
    std::vector<xmlrpc_c::value> waypoint_rows = waypoints_array.vectorValueValue();

    std::vector<std::array<double, 16>> waypoints;
    waypoints.reserve(waypoint_rows.size());

    for (const auto& row_value : waypoint_rows) {
        xmlrpc_c::value_array row_array(row_value);
        std::vector<xmlrpc_c::value> row = row_array.vectorValueValue();

        if (row.size() != 16) {
            throw std::runtime_error("Each Cartesian waypoint must have size 16");
        }

        std::array<double, 16> waypoint{};
        for (size_t i = 0; i < 16; ++i) {
            waypoint[i] = static_cast<double>(xmlrpc_c::value_double(row[i]));
        }
        waypoints.push_back(waypoint);
    }

    int samples_per_segment = paramList.getInt(1);
    std::string mode_str = paramList.getString(2);

    Pathmode path_mode;
    if (mode_str == "linear") {
        path_mode = Pathmode::Linear;
    } else if (mode_str == "catmull_rom") {
        path_mode = Pathmode::CatmullRomSpline;
    } else {
        throw std::runtime_error("path_mode must be 'linear' or 'catmull_rom'");
    }

    xmlrpc_c::value_array stiffness_array = paramList.getArray(3);
    std::vector<xmlrpc_c::value> stiffness_rows = stiffness_array.vectorValueValue();

    if (stiffness_rows.size() != 6) {
        throw std::runtime_error("Cartesian stiffness must have 6 rows");
    }

    std::array<std::array<double, 6>, 6> stiffness{};
    for (size_t i = 0; i < 6; ++i) {
        xmlrpc_c::value_array row_array(stiffness_rows[i]);
        std::vector<xmlrpc_c::value> row = row_array.vectorValueValue();

        if (row.size() != 6) {
            throw std::runtime_error("Each Cartesian stiffness row must have 6 elements");
        }

        for (size_t j = 0; j < 6; ++j) {
            stiffness[i][j] = static_cast<double>(xmlrpc_c::value_double(row[j]));
        }
    }

    double vmax_linear = paramList.getDouble(4);
    double acc_max_linear = paramList.getDouble(5);
    double vmax_angular = paramList.getDouble(6);
    double acc_max_angular = paramList.getDouble(7);
    double nullspace_stiffness = paramList.getDouble(8);

    // acc_max_* is accepted to keep the RPC shape parallel to moveCartesian.
    // moveC_path uses the runtime motion slowdown factor and the 1 kHz base
    // period for replay timing.
    paramList.verifyEnd(9);

    MoveCartesianPathTask task{};
    task.waypoints = std::move(waypoints);
    task.samples_per_segment = samples_per_segment;
    task.path_mode = path_mode;
    task.stiffness = stiffness;
    task.vmax_linear = vmax_linear;
    task.acc_max_linear = acc_max_linear;
    task.vmax_angular = vmax_angular;
    task.acc_max_angular = acc_max_angular;
    task.nullspace_stiffness = nullspace_stiffness;
    return task;
}


moveJ_method::moveJ_method(Robotmanager& manager) : manager_(manager) {
    this->_signature = "i:AAAA";
    this->_help =
        "Queue moveJ. Args: q_d[7] rad, stiffness[7][7], "
        "vmax[7] rad/s, acc_max[7] rad/s^2. Returns accept code.";
}
moveJ_method_no_queue::moveJ_method_no_queue(Robotmanager& manager)
    : manager_(manager) {
    this->_signature = "i:AAAA";
    this->_help =
        "Execute moveJ directly. Args: q_d[7] rad, stiffness[7][7], "
        "vmax[7] rad/s, acc_max[7] rad/s^2. Returns result code.";
}

moveJ_path_method::moveJ_path_method(Robotmanager& manager) : manager_(manager) {
    this->_signature = "i:AiSAAA";
    this->_help =
        "Queue moveJPath. Args: waypoints[N][7] rad, samples_per_segment, "
        "path_mode, stiffness[7][7], vmax[7] rad/s, acc_max[7] ignored. "
        "Runtime slowdown is controlled by setSlowdownFactor.";
}

move_cartesian_path_method::move_cartesian_path_method(Robotmanager& manager)
    : manager_(manager) {
    this->_signature = "i:AiSAddddd";
    this->_help =
        "Queue moveCartesianPath. Args: waypoints[N][16] row-major poses, "
        "samples_per_segment, path_mode, stiffness[6][6], vmax_linear m/s, "
        "acc_max_linear ignored, vmax_angular rad/s, acc_max_angular ignored, "
        "nullspace_stiffness. Runtime slowdown is controlled by "
        "setSlowdownFactor.";
}

void move_cartesian_path_method::execute(
    xmlrpc_c::paramList const& paramList,
    xmlrpc_c::value* const resultp) {

    MoveCartesianPathTask task = parse_move_cartesian_path_task(paramList);
    manager_.enqueueMoveCartesianPath(task);
    *resultp = xmlrpc_c::value_int(RPC_OK);
}

move_cartesian_path_no_queue_method::move_cartesian_path_no_queue_method(
    Robotmanager& manager)
    : manager_(manager) {
    this->_signature = "i:AiSAddddd";
    this->_help =
        "Execute moveCartesianPath directly. Args: waypoints[N][16] row-major "
        "poses, samples_per_segment, path_mode, stiffness[6][6], "
        "vmax_linear m/s, acc_max_linear ignored, vmax_angular rad/s, "
        "acc_max_angular ignored, nullspace_stiffness. Runtime slowdown is "
        "controlled by setSlowdownFactor.";
}

void move_cartesian_path_no_queue_method::execute(
    xmlrpc_c::paramList const& paramList,
    xmlrpc_c::value* const resultp) {

    MoveCartesianPathTask task = parse_move_cartesian_path_task(paramList);
    int result = manager_.executeMoveCartesianPath(task);
    *resultp = xmlrpc_c::value_int(result);
}


void moveJ_path_method::execute(xmlrpc_c::paramList const& paramList,
                                xmlrpc_c::value* const resultp) {
    MoveJPathTask task = parse_movej_path_task(paramList);
    manager_.enqueueMoveJPath(task);
    *resultp = xmlrpc_c::value_int(0);
}

moveJ_path_no_queue_method::moveJ_path_no_queue_method(Robotmanager& manager)
    : manager_(manager) {
    this->_signature = "i:AiSAAA";
    this->_help =
        "Execute moveJPath directly. Args: waypoints[N][7] rad, "
        "samples_per_segment, path_mode, stiffness[7][7], vmax[7] rad/s, "
        "acc_max[7] ignored. Runtime slowdown is controlled by "
        "setSlowdownFactor.";
}

void moveJ_path_no_queue_method::execute(xmlrpc_c::paramList const& paramList,
                                         xmlrpc_c::value* const resultp) {
    MoveJPathTask task = parse_movej_path_task(paramList);
    int result = manager_.executeMoveJPath(task);
    *resultp = xmlrpc_c::value_int(result);
}



void move_cartesian_method::execute(xmlrpc_c::paramList const& paramList,
                                    xmlrpc_c::value* const resultp) {
    MoveCartesianTask task = parse_move_cartesian_task(paramList);
    manager_.enqueueMoveCartesian(task);
    *resultp = xmlrpc_c::value_int(0);
}

void move_cartesian_no_queue_method::execute(xmlrpc_c::paramList const& paramList,
                                             xmlrpc_c::value* const resultp) {
    MoveCartesianTask task = parse_move_cartesian_task(paramList);
    int result = manager_.executeMoveCartesian(task);
    *resultp = xmlrpc_c::value_int(result);
}




move_cartesian_method::move_cartesian_method(Robotmanager& manager)
    : manager_(manager) {
    this->_signature = "i:AAddddd";
    this->_help =
        "Queue moveCartesian. Args: T_d[16] row-major pose in base/world frame, "
        "stiffness[6][6], vmax_linear m/s, acc_max_linear m/s^2, "
        "vmax_angular rad/s, acc_max_angular rad/s^2, nullspace_stiffness.";
}

move_cartesian_no_queue_method::move_cartesian_no_queue_method(Robotmanager& manager)
    : manager_(manager) {
    this->_signature = "i:AAddddd";
    this->_help =
        "Execute moveCartesian directly. Args: T_d[16] row-major pose in "
        "base/world frame, stiffness[6][6], vmax_linear m/s, "
        "acc_max_linear m/s^2, vmax_angular rad/s, acc_max_angular rad/s^2, "
        "nullspace_stiffness.";
}


void moveJ_method::execute(xmlrpc_c::paramList const& paramList,
                           xmlrpc_c::value* const resultp) {
    MoveJTask task = parse_movej_task(paramList);
    manager_.enqueueMoveJ(task);
    *resultp = xmlrpc_c::value_int(0);
}

void moveJ_method_no_queue::execute(xmlrpc_c::paramList const& paramList,
                                    xmlrpc_c::value* const resultp) {
    MoveJTask task = parse_movej_task(paramList);
    int result = manager_.executeMoveJ(task);
    *resultp = xmlrpc_c::value_int(result);
}




grasp_method::grasp_method(Robotmanager& manager) : manager_(manager) {
    this->_signature = "i:ddddd";
    this->_help =
        "Queue graspO. Args: width m, speed m/s, force N, "
        "epsilon_inner m, epsilon_outer m. Returns accept code.";
}

void grasp_method::execute(xmlrpc_c::paramList const& paramList,
                           xmlrpc_c::value* const resultp) {
    double width = paramList.getDouble(0);
    double speed = paramList.getDouble(1);
    double force = paramList.getDouble(2);
    double epsilon_inner = paramList.getDouble(3);
    double epsilon_outer = paramList.getDouble(4);

    paramList.verifyEnd(5);

    GripperTask task{
        GripperCommand::GRASP,
        width,
        speed,
        force,
        epsilon_inner,
        epsilon_outer
    };

    manager_.enqueueGripper(task);
    *resultp = xmlrpc_c::value_int(0);
}

get_gripper_state_method::get_gripper_state_method(Robotmanager& manager)
    : manager_(manager) {
    this->_signature = "i:";
    this->_help = "Get current gripper state enum value. Args: none.";
}

get_arm_state_method::get_arm_state_method(Robotmanager& manager)
    : manager_(manager) {
    this->_signature = "i:";
    this->_help = "Get current arm state enum value. Args: none.";
}

void get_arm_state_method::execute(xmlrpc_c::paramList const& paramList,
                                  xmlrpc_c::value* const resultp) {
    paramList.verifyEnd(0);
    arm_state state = manager_.arm_status.state.load();
    *resultp = xmlrpc_c::value_int(static_cast<int>(state));
}


void get_gripper_state_method::execute(xmlrpc_c::paramList const& paramList,
                                       xmlrpc_c::value* const resultp) {
    paramList.verifyEnd(0);
    gripper_state state = manager_.gripper_status.state.load();
    *resultp = xmlrpc_c::value_int(static_cast<int>(state));
}

gripper_home_method::gripper_home_method(Robotmanager& manager)
    : manager_(manager) {
    this->_signature = "i:";
    this->_help = "Queue gripper homing. Args: none. Returns accept code.";
}

void gripper_home_method::execute(xmlrpc_c::paramList const& paramList,
                                  xmlrpc_c::value* const resultp) {
    paramList.verifyEnd(0);

    GripperTask task{};
    task.command = GripperCommand::HOME;

    manager_.enqueueGripper(task);
    *resultp = xmlrpc_c::value_int(0);
}

gripper_release_method::gripper_release_method(Robotmanager& manager)
    : manager_(manager) {
    this->_signature = "i:d";
    this->_help =
        "Queue gripper release/open. Args: speed m/s. Returns accept code.";
}

void gripper_release_method::execute(xmlrpc_c::paramList const& paramList,
                                     xmlrpc_c::value* const resultp) {
    double speed = paramList.getDouble(0);
    paramList.verifyEnd(1);

    GripperTask task{};
    task.command = GripperCommand::RELEASE;
    task.speed = speed;

    manager_.enqueueGripper(task);
    *resultp = xmlrpc_c::value_int(0);
}

init_session_method::init_session_method(Robotmanager& manager)
    : manager_(manager) {
    this->_signature = "i:siiAAAA";
    this->_help =
        "Initialize session. Args: udp_ip, udp_port, udp_frequency_hz, "
        "lower_torque[7], upper_torque[7], lower_force[6], upper_force[6].";
}

void init_session_method::execute(xmlrpc_c::paramList const& paramList,
                                  xmlrpc_c::value* const resultp) {
    std::string udp_ip = paramList.getString(0);
    int udp_port = paramList.getInt(1);
    int udp_frequency_hz = paramList.getInt(2);

    auto to_array7 = [](const xmlrpc_c::value_array& arr) {
        std::vector<xmlrpc_c::value> values = arr.vectorValueValue();
        if (values.size() != 7) {
            throw std::runtime_error("Expected array of size 7");
        }
        std::array<double, 7> out{};
        for (size_t i = 0; i < 7; ++i) {
            out[i] = static_cast<double>(xmlrpc_c::value_double(values[i]));
        }
        return out;
    };

    auto to_array6 = [](const xmlrpc_c::value_array& arr) {
        std::vector<xmlrpc_c::value> values = arr.vectorValueValue();
        if (values.size() != 6) {
            throw std::runtime_error("Expected array of size 6");
        }
        std::array<double, 6> out{};
        for (size_t i = 0; i < 6; ++i) {
            out[i] = static_cast<double>(xmlrpc_c::value_double(values[i]));
        }
        return out;
    };

    std::array<double, 7> lower_torque = to_array7(paramList.getArray(3));
    std::array<double, 7> upper_torque = to_array7(paramList.getArray(4));
    std::array<double, 6> lower_force = to_array6(paramList.getArray(5));
    std::array<double, 6> upper_force = to_array6(paramList.getArray(6));

    paramList.verifyEnd(7);

    manager_.initSession(
        udp_ip,
        udp_port,
        udp_frequency_hz,
        lower_torque,
        upper_torque,
        lower_force,
        upper_force
    );

    *resultp = xmlrpc_c::value_int(0);
}

grasp_no_queue_method::grasp_no_queue_method(Robotmanager& manager)
    : manager_(manager) {
    this->_signature = "i:ddddd";
    this->_help =
        "Execute graspO directly. Args: width m, speed m/s, force N, "
        "epsilon_inner m, epsilon_outer m. Returns result code.";
}

void grasp_no_queue_method::execute(xmlrpc_c::paramList const& paramList,
                                    xmlrpc_c::value* const resultp) {
    double width = paramList.getDouble(0);
    double speed = paramList.getDouble(1);
    double force = paramList.getDouble(2);
    double epsilon_inner = paramList.getDouble(3);
    double epsilon_outer = paramList.getDouble(4);

    paramList.verifyEnd(5);

    GripperTask task{
        GripperCommand::GRASP,
        width,
        speed,
        force,
        epsilon_inner,
        epsilon_outer
    };

    int result = manager_.executeGripperTask(task);
    *resultp = xmlrpc_c::value_int(result);
}

gripper_home_no_queue_method::gripper_home_no_queue_method(Robotmanager& manager)
    : manager_(manager) {
    this->_signature = "i:";
    this->_help = "Execute gripper homing directly. Args: none.";
}

void gripper_home_no_queue_method::execute(xmlrpc_c::paramList const& paramList,
                                           xmlrpc_c::value* const resultp) {
    paramList.verifyEnd(0);

    GripperTask task{};
    task.command = GripperCommand::HOME;

    int result = manager_.executeGripperTask(task);
    *resultp = xmlrpc_c::value_int(result);
}

gripper_release_no_queue_method::gripper_release_no_queue_method(Robotmanager& manager)
    : manager_(manager) {
    this->_signature = "i:d";
    this->_help =
        "Execute gripper release/open directly. Args: speed m/s.";
}

void gripper_release_no_queue_method::execute(xmlrpc_c::paramList const& paramList,
                                              xmlrpc_c::value* const resultp) {
    double speed = paramList.getDouble(0);
    paramList.verifyEnd(1);

    GripperTask task{};
    task.command = GripperCommand::RELEASE;
    task.speed = speed;

    int result = manager_.executeGripperTask(task);
    *resultp = xmlrpc_c::value_int(result);
}

get_gripper_width_method::get_gripper_width_method(Robotmanager& manager)
    : manager_(manager) {
    this->_signature = "d:";
    this->_help = "Get cached gripper width in meters. Args: none.";
}

void get_gripper_width_method::execute(
    xmlrpc_c::paramList const& paramList,
    xmlrpc_c::value* const resultp
) {
    paramList.verifyEnd(0);

    double width = manager_.gripper_status.current_width.load();
    *resultp = xmlrpc_c::value_double(width);
}


set_idle_state_poll_frequency_method::set_idle_state_poll_frequency_method(
    Robotmanager& manager)
    : manager_(manager) {
    this->_signature = "i:i";
    this->_help =
        "Set idle robot state polling frequency. Args: frequency_hz.";
}

void set_idle_state_poll_frequency_method::execute(
    xmlrpc_c::paramList const& paramList,
    xmlrpc_c::value* const resultp) {
    int frequency_hz = paramList.getInt(0);
    paramList.verifyEnd(1);

    manager_.setIdleStatePollFrequency(frequency_hz);

    *resultp = xmlrpc_c::value_int(RPC_OK);
}

set_slowdown_factor_method::set_slowdown_factor_method(
    Robotmanager& manager)
    : manager_(manager) {
    this->_signature = "i:d";
    this->_help =
        "Set runtime motion slowdown factor. Args: slowdown_factor >= 1.0.";
}

void set_slowdown_factor_method::execute(
    xmlrpc_c::paramList const& paramList,
    xmlrpc_c::value* const resultp) {
    double slowdown_factor = paramList.getDouble(0);
    paramList.verifyEnd(1);

    int result = manager_.setMotionSlowdownFactor(slowdown_factor);
    *resultp = xmlrpc_c::value_int(result);
}

get_slowdown_factor_method::get_slowdown_factor_method(
    Robotmanager& manager)
    : manager_(manager) {
    this->_signature = "d:";
    this->_help = "Get runtime motion slowdown factor. Args: none.";
}

void get_slowdown_factor_method::execute(
    xmlrpc_c::paramList const& paramList,
    xmlrpc_c::value* const resultp) {
    paramList.verifyEnd(0);

    double slowdown_factor = manager_.getMotionSlowdownFactor();
    *resultp = xmlrpc_c::value_double(slowdown_factor);
}

start_trackj_method::start_trackj_method(Robotmanager& manager)
    : manager_(manager) {
    this->_signature = "i:idA";
    this->_help =
        "Start trackJ streaming control. Args: command_port, stream_hz, "
        "stiffness[7][7]. q_ref samples are sent over UDP.";
}

void start_trackj_method::execute(
    xmlrpc_c::paramList const& paramList,
    xmlrpc_c::value* const resultp) {

    TrackJTask task = parse_trackj_task(paramList);
    int result = manager_.startTrackJ(task);
    *resultp = xmlrpc_c::value_int(result);
}

start_trackj2_method::start_trackj2_method(Robotmanager& manager)
    : manager_(manager) {
    this->_signature = "i:idA";
    this->_help =
        "Start trackJ2 async target control. Args: command_port, stream_hz, "
        "stiffness[7][7]. q_target samples are sent over UDP.";
}

void start_trackj2_method::execute(
    xmlrpc_c::paramList const& paramList,
    xmlrpc_c::value* const resultp) {

    TrackJTask task = parse_trackj_task(paramList);
    int result = manager_.startTrackJ2(task);
    *resultp = xmlrpc_c::value_int(result);
}

start_trackc_method::start_trackc_method(Robotmanager& manager)
    : manager_(manager) {
    this->_signature = "i:idAd";
    this->_help =
        "Start trackC streaming control. Args: command_port, stream_hz, "
        "stiffness[6][6], nullspace_stiffness. T_ref samples are sent over UDP.";
}

void start_trackc_method::execute(
    xmlrpc_c::paramList const& paramList,
    xmlrpc_c::value* const resultp) {

    TrackCTask task = parse_trackc_task(paramList);
    int result = manager_.startTrackC(task);
    *resultp = xmlrpc_c::value_int(result);
}

start_trackc2_method::start_trackc2_method(Robotmanager& manager)
    : manager_(manager) {
    this->_signature = "i:idAd";
    this->_help =
        "Start trackC2 async target control. Args: command_port, stream_hz, "
        "stiffness[6][6], nullspace_stiffness. T_target samples are sent over UDP.";
}

void start_trackc2_method::execute(
    xmlrpc_c::paramList const& paramList,
    xmlrpc_c::value* const resultp) {

    TrackCTask task = parse_trackc_task(paramList);
    int result = manager_.startTrackC2(task);
    *resultp = xmlrpc_c::value_int(result);
}

set_trackj2_filter_params_method::set_trackj2_filter_params_method(
    Robotmanager& manager)
    : manager_(manager) {
    this->_signature = "i:dd";
    this->_help = "Set TrackJ2 filter params. Args: omega_n, zeta.";
}

void set_trackj2_filter_params_method::execute(
    xmlrpc_c::paramList const& paramList,
    xmlrpc_c::value* const resultp) {
    double omega_n = paramList.getDouble(0);
    double zeta = paramList.getDouble(1);
    paramList.verifyEnd(2);

    int result = manager_.setTrackJ2FilterParams(omega_n, zeta);
    *resultp = xmlrpc_c::value_int(result);
}

set_trackc2_filter_params_method::set_trackc2_filter_params_method(
    Robotmanager& manager)
    : manager_(manager) {
    this->_signature = "i:dd";
    this->_help = "Set TrackC2 filter params. Args: omega_n, zeta.";
}

void set_trackc2_filter_params_method::execute(
    xmlrpc_c::paramList const& paramList,
    xmlrpc_c::value* const resultp) {
    double omega_n = paramList.getDouble(0);
    double zeta = paramList.getDouble(1);
    paramList.verifyEnd(2);

    int result = manager_.setTrackC2FilterParams(omega_n, zeta);
    *resultp = xmlrpc_c::value_int(result);
}


recover_system_method::recover_system_method(Robotmanager& manager)
    : manager_(manager) {
    this->_signature = "i:";
    this->_help = "Recover arm/gripper manager state. Args: none.";
}

void recover_system_method::execute(xmlrpc_c::paramList const& paramList,
                                    xmlrpc_c::value* const resultp) {
    paramList.verifyEnd(0);
    int result = manager_.recoverSystem();
    *resultp = xmlrpc_c::value_int(result);
}

stop_arm_motion_method::stop_arm_motion_method(Robotmanager& manager)
    : manager_(manager) {
    this->_signature = "i:";
    this->_help =
        "Stop current arm motion and clear queued arm commands. Args: none.";
}

void stop_arm_motion_method::execute(
    xmlrpc_c::paramList const& paramList,
    xmlrpc_c::value* const resultp) {
    paramList.verifyEnd(0);
    int result = manager_.stopArmMotion();
    *resultp = xmlrpc_c::value_int(result);
}




void register_methods(xmlrpc_c::registry& registry, Robotmanager& manager) {
    //queue methods
    registry.addMethod("moveJ_queue", xmlrpc_c::methodPtr(new moveJ_method(manager)));
    registry.addMethod("graspO_queue", xmlrpc_c::methodPtr(new grasp_method(manager)));
    registry.addMethod("moveJPath_queue", xmlrpc_c::methodPtr(new moveJ_path_method(manager)));
    registry.addMethod("gripperHome_queue", xmlrpc_c::methodPtr(new gripper_home_method(manager)));
    registry.addMethod("gripperRelease_queue", xmlrpc_c::methodPtr(new gripper_release_method(manager)));
    registry.addMethod("moveCartesian_queue", xmlrpc_c::methodPtr(new move_cartesian_method(manager)));
    registry.addMethod("moveCartesianPath_queue", xmlrpc_c::methodPtr(new move_cartesian_path_method(manager)));

    //no queue
    registry.addMethod("moveJ",xmlrpc_c::methodPtr(new moveJ_method_no_queue(manager)));
    registry.addMethod("graspO", xmlrpc_c::methodPtr(new grasp_no_queue_method(manager)));
    registry.addMethod("gripperHome", xmlrpc_c::methodPtr(new gripper_home_no_queue_method(manager)));
    registry.addMethod("gripperRelease", xmlrpc_c::methodPtr(new gripper_release_no_queue_method(manager)));
    registry.addMethod("moveCartesian", xmlrpc_c::methodPtr(new move_cartesian_no_queue_method(manager)));
    registry.addMethod("moveJPath", xmlrpc_c::methodPtr(new moveJ_path_no_queue_method(manager)));
    registry.addMethod("moveCartesianPath", xmlrpc_c::methodPtr(new move_cartesian_path_no_queue_method(manager)));

    //state, session, recovery
    registry.addMethod("getGripperState", xmlrpc_c::methodPtr(new get_gripper_state_method(manager)));
    registry.addMethod("getArmState", xmlrpc_c::methodPtr(new get_arm_state_method(manager)));
    registry.addMethod("initSession", xmlrpc_c::methodPtr(new init_session_method(manager)));
    registry.addMethod("recoverSystem", xmlrpc_c::methodPtr(new recover_system_method(manager)));
    registry.addMethod("setIdleStatePollFrequency", xmlrpc_c::methodPtr(new set_idle_state_poll_frequency_method(manager)));
    registry.addMethod("setSlowdownFactor", xmlrpc_c::methodPtr(new set_slowdown_factor_method(manager)));
    registry.addMethod("getSlowdownFactor", xmlrpc_c::methodPtr(new get_slowdown_factor_method(manager)));
    registry.addMethod("startTrackJ", xmlrpc_c::methodPtr(new start_trackj_method(manager)));
    registry.addMethod("startTrackC", xmlrpc_c::methodPtr(new start_trackc_method(manager)));
    registry.addMethod("startTrackJ2", xmlrpc_c::methodPtr(new start_trackj2_method(manager)));
    registry.addMethod("startTrackC2", xmlrpc_c::methodPtr(new start_trackc2_method(manager)));
    registry.addMethod("setTrackJ2FilterParams", xmlrpc_c::methodPtr(new set_trackj2_filter_params_method(manager)));
    registry.addMethod("setTrackC2FilterParams", xmlrpc_c::methodPtr(new set_trackc2_filter_params_method(manager)));
    registry.addMethod("stopArmMotion", xmlrpc_c::methodPtr(new stop_arm_motion_method(manager)));
    registry.addMethod("getGripperWidth", xmlrpc_c::methodPtr(new get_gripper_width_method(manager)));
}
