from remote_controller.RemoteControllerClient import RemoteControllerClient
from remote_controller.robot_kinematics import RobotModel

import math
import time
import numpy as np


SERVER_URL = "http://localhost:8008/RPC2"
UDP_IP = "127.0.0.1"
UDP_PORT = 9000
UDP_FREQUENCY_HZ = 500
BUFFER_CAPACITY = 30000

PATH_MODE = "linear"

PATH_DURATION_S = 30.0
PATH_WAYPOINT_COUNT = 24
SAMPLES_PER_SEGMENT = int(PATH_DURATION_S / PATH_WAYPOINT_COUNT / 0.001)
SLOWDOWN_FACTOR = 1.0

STOP_AFTER_S = 8.0
WAIT_TIMEOUT_S = 40.0

ARC_RADIUS_M = 0.03

OPEN_PATH_DISTANCE_M = 0.12
OPEN_PATH_LIFT_M = 0.04

STOP_FINAL_TARGET_MIN_ERROR_M = 0.04
RECOVERY_FINAL_TARGET_TOL_M = 0.020
RECOVERY_FINAL_TARGET_TOL_RAD = 0.12


def make_cartesian_stiffness():
    return [
        [150.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 150.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 150.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 12.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 12.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 12.0],
    ]


def flatten_pose(T):
    return [float(x) for x in T.reshape(-1)]


def rotation_angle(R1, R2):
    R_err = R2 @ R1.T
    value = (np.trace(R_err) - 1.0) / 2.0
    value = max(-1.0, min(1.0, value))
    return math.acos(value)


def pose_error(T_current, T_target):
    p_current = T_current[:3, 3]
    p_target = T_target[:3, 3]

    R_current = T_current[:3, :3]
    R_target = T_target[:3, :3]

    pos_err_m = float(np.linalg.norm(p_target - p_current))
    rot_err_rad = rotation_angle(R_current, R_target)

    return pos_err_m, rot_err_rad


def print_pose_translation(label, T):
    p = T[:3, 3]
    print(
        f"{label}: "
        f"x={p[0]:+.4f} m, y={p[1]:+.4f} m, z={p[2]:+.4f} m"
    )


def make_open_arc_waypoints(
    T_start,
    direction=1.0,
    distance_m=OPEN_PATH_DISTANCE_M,
    lift_m=OPEN_PATH_LIFT_M,
):
    waypoints = []

    for i in range(1, PATH_WAYPOINT_COUNT + 1):
        phase = i / PATH_WAYPOINT_COUNT

        # Smooth 0 -> 1. Final x is far away from start.
        s = 0.5 - 0.5 * math.cos(math.pi * phase)

        dx_m = direction * distance_m * s

        # Lift in the middle, return to original z at the end.
        dz_m = lift_m * math.sin(math.pi * phase)

        T = np.array(T_start, dtype=float).copy()
        T[0, 3] += dx_m
        T[2, 3] += dz_m

        waypoints.append(flatten_pose(T))

    return waypoints


def make_closed_arc_waypoints(T_start, direction=1.0, radius_m=ARC_RADIUS_M):
    waypoints = []

    for i in range(1, PATH_WAYPOINT_COUNT + 1):
        t = PATH_DURATION_S * i / PATH_WAYPOINT_COUNT

        angle = math.pi / 4.0 * (
            1.0 - math.cos(2.0 * math.pi * t / PATH_DURATION_S)
        )

        dx_m = direction * radius_m * math.sin(angle)
        dz_m = radius_m * (1.0 - math.cos(angle))

        T = np.array(T_start, dtype=float).copy()
        T[0, 3] += dx_m
        T[2, 3] += dz_m

        waypoints.append(flatten_pose(T))

    return waypoints


def make_recovery_waypoints(T_start):
    waypoints = []

    for i in range(1, 21):
        phase = i / 20.0

        # Small smooth up-and-back motion after stop.
        dz_m = 0.015 * math.sin(math.pi * phase)

        T = np.array(T_start, dtype=float).copy()
        T[2, 3] += dz_m

        waypoints.append(flatten_pose(T))

    return waypoints


def matrix_from_flat_pose(flat_pose):
    return np.array(flat_pose, dtype=float).reshape(4, 4)


def ensure_ok(client, result, name):
    client.print_rpc_result(result, name)
    if result != 0:
        raise RuntimeError(
            f"{name} failed: {result} ({client.decode_rpc_result(result)})"
        )


def main():
    robot_model = RobotModel()

    client = RemoteControllerClient(
        SERVER_URL,
        capacity=BUFFER_CAPACITY,
        horizon_prev=1,
        sensor_size=22,
        default_frame_name="panda_hand_tcp",
    )

    try:
        result = client.init(
            udp_ip=UDP_IP,
            udp_port=UDP_PORT,
            udp_frequency_hz=UDP_FREQUENCY_HZ,
            recover_before_init=True,
        )
        ensure_ok(client, result, "init")

        stiffness = make_cartesian_stiffness()

        path_vmax_linear_m_s = 0.04
        path_vmax_angular_rad_s = 0.15
        nullspace_stiffness = 0.0

        T0 = client.get_current_tcp_pose(robot_model, flush=True)
        print_pose_translation("start TCP", T0)

        print("\nQueue 3 moveC_path commands")
        client.set_slowdown_factor(SLOWDOWN_FACTOR)
        first_path_waypoints = None

        for index, direction in enumerate([1.0, -1.0, 1.0], start=1):
            if index == 1:
                # First path must be open and long, otherwise stop test can be
                # confused by a closed path ending near the start.
                waypoints = make_open_arc_waypoints(
                    T0,
                    direction=direction,
                    distance_m=OPEN_PATH_DISTANCE_M,
                    lift_m=OPEN_PATH_LIFT_M,
                )
            else:
                waypoints = make_closed_arc_waypoints(
                    T0,
                    direction=direction,
                    radius_m=ARC_RADIUS_M,
                )

            if index == 1:
                first_path_waypoints = waypoints

            result = client.movecart(
                waypoints,
                stiffness=stiffness,
                vmax_linear=path_vmax_linear_m_s,
                vmax_angular=path_vmax_angular_rad_s,
                nullspace_stiffness=nullspace_stiffness,
                queue=True,
                samples_per_segment=SAMPLES_PER_SEGMENT,
                path_mode=PATH_MODE,
            )
            ensure_ok(client, result, f"queue moveC_path {index}")

        first_final_T = matrix_from_flat_pose(first_path_waypoints[-1])

        print(
            f"\nSleep {STOP_AFTER_S:.1f}s, then send stopArmMotion "
            "while first open path should be executing"
        )
        time.sleep(STOP_AFTER_S)

        stop_result = client.stop_arm_motion()
        ensure_ok(client, stop_result, "stopArmMotion")

        arm_state_after_stop = client.wait_until_arm_idle_ok(timeout=WAIT_TIMEOUT_S)
        print(f"arm state after stop: {arm_state_after_stop}")

        time.sleep(0.2)
        T_after_stop = client.get_current_tcp_pose(robot_model, flush=True)
        print_pose_translation("TCP after stop", T_after_stop)

        stop_pos_err_m, stop_rot_err_rad = pose_error(T_after_stop, first_final_T)
        print(
            "stop check vs first final target: "
            f"pos_error={stop_pos_err_m:.5f} m, "
            f"rot_error={stop_rot_err_rad:.5f} rad"
        )

        if stop_pos_err_m < STOP_FINAL_TARGET_MIN_ERROR_M:
            raise RuntimeError(
                "Stop check failed: TCP is too close to first path final target. "
                "The motion may have finished instead of stopping early."
            )

        print("PASS: stopArmMotion stopped before first path final target.")

        print("\nQueue one new moveC_path after stop")
        recovery_waypoints = make_recovery_waypoints(T_after_stop)
        recovery_final_T = matrix_from_flat_pose(recovery_waypoints[-1])

        result = client.movecart(
            recovery_waypoints,
            stiffness=stiffness,
            vmax_linear=0.03,
            vmax_angular=0.10,
            nullspace_stiffness=nullspace_stiffness,
            queue=True,
            samples_per_segment=200,
            path_mode=PATH_MODE,
        )
        ensure_ok(client, result, "queue recovery moveC_path")

        arm_state_after_recovery = client.wait_until_arm_idle_ok(timeout=WAIT_TIMEOUT_S)
        print(f"arm state after recovery command: {arm_state_after_recovery}")

        if arm_state_after_recovery == "ERROR":
            raise RuntimeError("Arm entered ERROR during recovery moveC_path.")

        time.sleep(0.2)
        T_after_recovery = client.get_current_tcp_pose(robot_model, flush=True)
        print_pose_translation("TCP after recovery", T_after_recovery)

        recovery_pos_err_m, recovery_rot_err_rad = pose_error(
            T_after_recovery,
            recovery_final_T,
        )
        print(
            "recovery final check: "
            f"pos_error={recovery_pos_err_m:.5f} m, "
            f"rot_error={recovery_rot_err_rad:.5f} rad"
        )

        if (
            recovery_pos_err_m > RECOVERY_FINAL_TARGET_TOL_M
            or recovery_rot_err_rad > RECOVERY_FINAL_TARGET_TOL_RAD
        ):
            raise RuntimeError(
                "Recovery moveC_path did not reach final target. "
                f"pos_error={recovery_pos_err_m:.5f} m, "
                f"rot_error={recovery_rot_err_rad:.5f} rad."
            )

        print("PASS: recovery moveC_path succeeded after stop.")
        print("\nTest finished successfully.")

    finally:
        client.close()


if __name__ == "__main__":
    main()
