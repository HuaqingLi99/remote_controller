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

ENABLE_IDLE_POLL = False
IDLE_POLL_HZ = 1

PATH_MODE = "catmull_rom"

FIXED_START_Q_RAD = [
    -0.000254212,
    -0.784373,
    0.000115828,
    -2.35741,
    0.000464327,
    1.57113,
    0.785704,
]

PATH_DURATION_S = 10.0
PATH_WAYPOINT_COUNT = 40
SAMPLES_PER_SEGMENT = int(PATH_DURATION_S / PATH_WAYPOINT_COUNT / 0.001)
SLOWDOWN_FACTOR = 1.0
ARC_RADIUS_M = 0.1

MAX_START_MOVE_M = 0.20
START_REACHED_POS_TOL_M = 0.015
START_REACHED_ROT_TOL_RAD = 0.08

POSITION_REACHED_TOL_M = 0.035
ROTATION_REACHED_TOL_RAD = 0.15
MIN_TCP_Z_M = 0.20


def make_cartesian_stiffness():
    return [
        [850.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 850.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 850.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 50.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 50.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 50.0],
    ]


def flatten_pose(T):
    return [float(x) for x in T.reshape(-1)]


def make_fixed_start_pose(client, robot_model):
    return client.get_tcp_pose_from_q(robot_model, FIXED_START_Q_RAD)


def make_arc_waypoints(T_start):
    waypoints = []
    min_z_m = float("inf")
    min_dz_m = float("inf")

    for i in range(1, PATH_WAYPOINT_COUNT + 1):
        t = PATH_DURATION_S * i / PATH_WAYPOINT_COUNT

        angle = math.pi / 4.0 * (
            1.0 - math.cos(2.0 * math.pi * t / PATH_DURATION_S)
        )

        dx_m = ARC_RADIUS_M * math.sin(angle)
        dz_m = ARC_RADIUS_M * (1.0 - math.cos(angle))

        T = np.array(T_start, dtype=float).copy()
        T[0, 3] += dx_m
        T[2, 3] += dz_m

        min_z_m = min(min_z_m, T[2, 3])
        min_dz_m = min(min_dz_m, dz_m)
        waypoints.append(flatten_pose(T))

    if min_dz_m < -1e-9:
        raise RuntimeError(
            f"Generated path contains downward dz: min_dz={min_dz_m:.6f} m."
        )

    if min_z_m < MIN_TCP_Z_M:
        raise RuntimeError(
            f"Generated arc goes too low: min_z={min_z_m:.3f} m, "
            f"MIN_TCP_Z_M={MIN_TCP_Z_M:.3f} m."
        )

    return waypoints


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


def print_pose_translation(name, T):
    p_m = T[:3, 3]
    p_cm = p_m * 100.0
    print(
        f"{name} translation: "
        f"[{p_m[0]:+.4f}, {p_m[1]:+.4f}, {p_m[2]:+.4f}] m "
        f"= [{p_cm[0]:+.1f}, {p_cm[1]:+.1f}, {p_cm[2]:+.1f}] cm"
    )


def assert_pose_close(T_current, T_target, name):
    pos_err_m, rot_err_rad = pose_error(T_current, T_target)
    print(
        f"{name} pose error: "
        f"pos={pos_err_m:.5f} m ({pos_err_m * 100.0:.2f} cm), "
        f"rot={rot_err_rad:.5f} rad"
    )

    if pos_err_m > START_REACHED_POS_TOL_M or rot_err_rad > START_REACHED_ROT_TOL_RAD:
        raise RuntimeError(
            f"{name} was not reached. Refuse to start moveC_path. "
            f"pos_err={pos_err_m:.5f} m, rot_err={rot_err_rad:.5f} rad."
        )


def get_udp_frames(client):
    buffer = client.udp_receiver.data_buffer
    with buffer.lock:
        return list(buffer.buffer)


def compute_tcp_poses_from_frames(client, frames, robot_model):
    poses = []

    for frame in frames:
        q = frame.data[:7]
        poses.append(client.get_tcp_pose_from_q(robot_model, q))

    return poses


def analyze_waypoints(client, frames, waypoints, robot_model):
    if not frames:
        print("no UDP frames captured")
        return

    tcp_poses = compute_tcp_poses_from_frames(client, frames, robot_model)
    timestamps = [frame.timestamp for frame in frames]
    t0 = timestamps[0]

    waypoint_poses = [
        np.array(waypoint, dtype=float).reshape(4, 4)
        for waypoint in waypoints
    ]

    print("\nCartesian waypoint check from UDP frames:")

    previous_best_index = 0
    check_indices = [
        0,
        len(waypoint_poses) // 4,
        len(waypoint_poses) // 2,
        3 * len(waypoint_poses) // 4,
        len(waypoint_poses) - 1,
    ]

    for waypoint_index in check_indices:
        waypoint_T = waypoint_poses[waypoint_index]

        best_index = None
        best_pos_error_m = float("inf")
        best_rot_error_rad = float("inf")
        best_score = float("inf")

        for i in range(previous_best_index, len(tcp_poses)):
            pos_err_m, rot_err_rad = pose_error(tcp_poses[i], waypoint_T)
            score = pos_err_m + 0.10 * rot_err_rad

            if score < best_score:
                best_score = score
                best_pos_error_m = pos_err_m
                best_rot_error_rad = rot_err_rad
                best_index = i

        reached = (
            best_pos_error_m <= POSITION_REACHED_TOL_M
            and best_rot_error_rad <= ROTATION_REACHED_TOL_RAD
        )

        best_time = timestamps[best_index] - t0 if best_index is not None else None

        if best_index is not None:
            previous_best_index = best_index

        print(
            f"waypoint {waypoint_index + 1:02d}: "
            f"pos_error={best_pos_error_m:.5f} m, "
            f"rot_error={best_rot_error_rad:.5f} rad, "
            f"closest_time={best_time:.3f}s, "
            f"reached={reached}"
        )


def print_path_summary(T_start, waypoints):
    poses = [
        np.array(waypoint, dtype=float).reshape(4, 4)
        for waypoint in waypoints
    ]

    p0 = T_start[:3, 3]
    positions = np.array([T[:3, 3] for T in poses])
    dp = positions - p0

    print("\nCartesian arc summary:")
    print(f"path duration target: {PATH_DURATION_S * SLOWDOWN_FACTOR:.2f} s")
    print(f"waypoints: {PATH_WAYPOINT_COUNT}")
    print(f"samples_per_segment: {SAMPLES_PER_SEGMENT}")
    print(f"slowdown_factor: {SLOWDOWN_FACTOR}")
    print(f"arc radius: {ARC_RADIUS_M:.3f} m ({ARC_RADIUS_M * 100.0:.1f} cm)")
    print(f"max forward dx: {dp[:, 0].max():.4f} m ({dp[:, 0].max() * 100.0:.2f} cm)")
    print(f"max upward dz: {dp[:, 2].max():.4f} m ({dp[:, 2].max() * 100.0:.2f} cm)")
    print(f"min dz: {dp[:, 2].min():.4f} m ({dp[:, 2].min() * 100.0:.2f} cm)")
    print(f"start z: {p0[2]:.4f} m ({p0[2] * 100.0:.2f} cm)")
    print(f"min path z: {positions[:, 2].min():.4f} m ({positions[:, 2].min() * 100.0:.2f} cm)")


def ensure_ok(client, result, name):
    print(f"{name} result: {result} ({client.decode_rpc_result(result)})")

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
    )

    if client.default_frame_name not in robot_model.list_frames():
        raise RuntimeError(
            f"{client.default_frame_name} not found in URDF."
        )

    try:
        result = client.init(
            udp_ip=UDP_IP,
            udp_port=UDP_PORT,
            udp_frequency_hz=UDP_FREQUENCY_HZ,
            recover_before_init=True,
        )
        ensure_ok(client, result, "init")

        if ENABLE_IDLE_POLL:
            result = client.set_idle_state_poll_frequency(IDLE_POLL_HZ)
            ensure_ok(client, result, "set idle poll")

        stiffness = make_cartesian_stiffness()

        start_vmax_linear_m_s = 0.03
        start_acc_max_linear_m_s2 = 0.06
        start_vmax_angular_rad_s = 0.15
        start_acc_max_angular_rad_s2 = 0.30

        path_vmax_linear_m_s = 0.05
        path_vmax_angular_rad_s = 0.20
        nullspace_stiffness = 0.0

        client.empty_buffer()
        state = client.wait_for_first_udp(timeout=2.0)
        q0 = state["q"]

        print("q0:")
        print([round(x, 4) for x in q0])

        T_current = client.get_tcp_pose_from_q(robot_model, q0)
        T_fixed_start = make_fixed_start_pose(client, robot_model)

        print("\nUnit check:")
        print("all Cartesian positions are meters. 0.03 m = 3.0 cm.")
        print(f"default TCP frame: {client.default_frame_name}")
        print_pose_translation("current TCP", T_current)
        print_pose_translation("fixed start TCP", T_fixed_start)

        start_move_distance_m, start_move_rotation_rad = pose_error(
            T_current,
            T_fixed_start,
        )
        print(
            "current -> fixed start distance: "
            f"{start_move_distance_m:.5f} m "
            f"({start_move_distance_m * 100.0:.2f} cm), "
            f"rotation={start_move_rotation_rad:.5f} rad"
        )

        if start_move_distance_m > MAX_START_MOVE_M:
            raise RuntimeError(
                f"Fixed start is too far from current TCP: "
                f"{start_move_distance_m:.5f} m. "
                f"MAX_START_MOVE_M={MAX_START_MOVE_M:.3f} m."
            )

        client.empty_buffer()
        print("\nmoveC: go to fixed start TCP pose")
        result = client.movecart(
            flatten_pose(T_fixed_start),
            stiffness=stiffness,
            vmax_linear=start_vmax_linear_m_s,
            acc_max_linear=start_acc_max_linear_m_s2,
            vmax_angular=start_vmax_angular_rad_s,
            acc_max_angular=start_acc_max_angular_rad_s2,
            nullspace_stiffness=nullspace_stiffness,
            queue=False,
        )
        ensure_ok(client, result, "moveC fixed start")

        client.empty_buffer()
        state_after_start = client.wait_for_first_udp(timeout=2.0)
        q_start_actual = state_after_start["q"]
        T_start_actual = client.get_tcp_pose_from_q(robot_model, q_start_actual)
        assert_pose_close(T_start_actual, T_fixed_start, "fixed start")

        waypoints = make_arc_waypoints(T_start_actual)
        print_path_summary(T_start_actual, waypoints)

        client.empty_buffer()
        client.set_slowdown_factor(SLOWDOWN_FACTOR)
        start_time = time.monotonic()

        print("\nmoveC path: slow +x/+z arc")
        result = client.movecart(
            waypoints,
            stiffness=stiffness,
            vmax_linear=path_vmax_linear_m_s,
            vmax_angular=path_vmax_angular_rad_s,
            nullspace_stiffness=nullspace_stiffness,
            queue=False,
            samples_per_segment=SAMPLES_PER_SEGMENT,
            path_mode=PATH_MODE,
        )
        ensure_ok(client, result, "moveC path")

        end_time = time.monotonic()
        elapsed = end_time - start_time

        frames = get_udp_frames(client)
        packet_count = len(frames)
        measured_hz = packet_count / elapsed if elapsed > 0.0 else 0.0
        expected_packets = UDP_FREQUENCY_HZ * elapsed

        print("\nUDP moving Cartesian path result:")
        print(f"packet_count: {packet_count}")
        print(f"elapsed_s: {elapsed:.3f}")
        print(f"target_hz: {UDP_FREQUENCY_HZ}")
        print(f"measured_hz: {measured_hz:.1f}")
        print(f"expected_packets: {expected_packets:.1f}")
        print(f"final arm state: {client.get_arm_state()}")

        analyze_waypoints(client, frames, waypoints, robot_model)

    finally:
        client.close()


if __name__ == "__main__":
    main()
