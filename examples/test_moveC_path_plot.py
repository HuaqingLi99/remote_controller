from remote_controller import RemoteControllerClient, RobotModel

import math
import time
from pathlib import Path

import numpy as np


try:
    import matplotlib.pyplot as plt
except ImportError:
    plt = None


SERVER_URL = "http://localhost:8008/RPC2"
UDP_IP = "127.0.0.1"
UDP_PORT = 9000

UDP_FREQUENCY_HZ = 500
BUFFER_CAPACITY = 30000

MODES = ["linear", "catmull_rom"]

FIXED_START_Q_RAD = [
    -0.000254212,
    -0.784373,
    0.000115828,
    -2.35741,
    0.000464327,
    1.57113,
    0.785704,
]

PATH_DURATION_S = 20.0
PATH_WAYPOINT_COUNT = 8
SAMPLES_PER_SEGMENT = int(PATH_DURATION_S / PATH_WAYPOINT_COUNT / 0.001)
SLOWDOWN_FACTOR = 1.0

ARC_RADIUS_M = 0.08

MAX_START_MOVE_M = 0.20
START_REACHED_POS_TOL_M = 0.020
START_REACHED_ROT_TOL_RAD = 0.10

POSITION_REACHED_TOL_M = 0.040
ROTATION_REACHED_TOL_RAD = 0.15
MIN_TCP_Z_M = 0.20

OUTPUT_DIR = Path(__file__).resolve().parent / "output"
PLOT_PATH = OUTPUT_DIR / "moveC_path_linear_vs_catmull.png"


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


def matrix_from_flat_pose(flat_pose):
    return np.array(flat_pose, dtype=float).reshape(4, 4)


def make_fixed_start_pose(client, robot_model):
    return client.get_tcp_pose_from_q(robot_model, FIXED_START_Q_RAD)


def make_corner_waypoints(T_start):
    offsets = [
        (0.025, 0.000, 0.000),
        (0.045, 0.000, 0.040),
        (0.070, 0.018, 0.045),
        (0.095, 0.018, 0.010),
        (0.120, 0.000, 0.000),
    ]

    waypoints = []
    min_z_m = float("inf")

    for dx_m, dy_m, dz_m in offsets:
        T = np.array(T_start, dtype=float).copy()
        T[0, 3] += dx_m
        T[1, 3] += dy_m
        T[2, 3] += dz_m

        min_z_m = min(min_z_m, T[2, 3])
        waypoints.append(flatten_pose(T))

    if min_z_m < MIN_TCP_Z_M:
        raise RuntimeError(
            f"Generated path goes too low: min_z={min_z_m:.3f} m, "
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


def ensure_ok(client, result, name):
    client.print_rpc_result(result, name)
    if result != 0:
        raise RuntimeError(
            f"{name} failed: {result} ({client.decode_rpc_result(result)})"
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
            f"{name} was not reached. "
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


def analyze_waypoint_errors(client, frames, waypoints, robot_model):
    if not frames:
        raise RuntimeError("No UDP frames captured.")

    tcp_poses = compute_tcp_poses_from_frames(client, frames, robot_model)
    timestamps = [frame.timestamp for frame in frames]
    t0 = timestamps[0]

    waypoint_poses = [matrix_from_flat_pose(waypoint) for waypoint in waypoints]

    pos_errors = []
    rot_errors = []
    best_times = []

    previous_best_index = 0

    for waypoint_T in waypoint_poses:
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

        if best_index is not None:
            previous_best_index = best_index
            best_times.append(timestamps[best_index] - t0)
        else:
            best_times.append(float("nan"))

        pos_errors.append(best_pos_error_m)
        rot_errors.append(best_rot_error_rad)

    return {
        "pos_errors": np.array(pos_errors),
        "rot_errors": np.array(rot_errors),
        "best_times": np.array(best_times),
        "tcp_poses": tcp_poses,
        "timestamps": np.array(timestamps) - t0,
    }


def summarize_mode(mode, result):
    pos_errors = result["pos_errors"]
    rot_errors = result["rot_errors"]

    print(f"\nSummary for {mode}:")
    print(f"mean waypoint pos error: {pos_errors.mean():.5f} m")
    print(f"max waypoint pos error: {pos_errors.max():.5f} m")
    print(f"mean waypoint rot error: {rot_errors.mean():.5f} rad")
    print(f"max waypoint rot error: {rot_errors.max():.5f} rad")


def move_to_start(client, robot_model, T_fixed_start, stiffness):
    result = client.movecart(
        flatten_pose(T_fixed_start),
        stiffness=stiffness,
        vmax_linear=0.03,
        acc_max_linear=0.06,
        vmax_angular=0.15,
        acc_max_angular=0.30,
        nullspace_stiffness=0.0,
        queue=False,
    )
    ensure_ok(client, result, "moveC fixed start")

    client.empty_buffer()
    state_after_start = client.wait_for_first_udp(timeout=2.0)
    q_start_actual = state_after_start["q"]
    T_start_actual = client.get_tcp_pose_from_q(robot_model, q_start_actual)

    assert_pose_close(T_start_actual, T_fixed_start, "fixed start")
    return T_start_actual


def run_mode(client, robot_model, mode, T_fixed_start, stiffness):
    print(f"\n=== Run mode: {mode} ===")

    T_start_actual = move_to_start(
        client,
        robot_model,
        T_fixed_start,
        stiffness,
    )

    waypoints = make_corner_waypoints(T_start_actual)

    client.empty_buffer()
    client.set_slowdown_factor(SLOWDOWN_FACTOR)
    start_time = time.monotonic()

    result = client.movecart(
        waypoints,
        stiffness=stiffness,
        vmax_linear=0.05,
        vmax_angular=0.20,
        nullspace_stiffness=0.0,
        queue=False,
        samples_per_segment=SAMPLES_PER_SEGMENT,
        path_mode=mode,
    )
    ensure_ok(client, result, f"moveC path {mode}")

    elapsed = time.monotonic() - start_time

    frames = get_udp_frames(client)
    packet_count = len(frames)
    measured_hz = packet_count / elapsed if elapsed > 0.0 else 0.0

    print(f"elapsed_s: {elapsed:.3f}")
    print(f"packet_count: {packet_count}")
    print(f"measured_hz: {measured_hz:.1f}")
    print(f"final arm state: {client.get_arm_state()}")

    analysis = analyze_waypoint_errors(client, frames, waypoints, robot_model)
    analysis["waypoints"] = waypoints
    analysis["elapsed_s"] = elapsed
    analysis["packet_count"] = packet_count
    analysis["measured_hz"] = measured_hz

    summarize_mode(mode, analysis)
    return analysis


def extract_xz_from_poses(poses):
    positions = np.array([T[:3, 3] for T in poses])
    return positions[:, 0], positions[:, 2]


def plot_results(results):
    if plt is None:
        print("\nmatplotlib not installed. Skip plot.")
        print("Install with: pip install matplotlib")
        return

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    linear = results["linear"]
    catmull = results["catmull_rom"]

    waypoint_poses = [matrix_from_flat_pose(wp) for wp in linear["waypoints"]]
    waypoint_positions = np.array([T[:3, 3] for T in waypoint_poses])

    linear_x, linear_z = extract_xz_from_poses(linear["tcp_poses"])
    catmull_x, catmull_z = extract_xz_from_poses(catmull["tcp_poses"])

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    waypoint_indices = np.arange(1, len(waypoint_positions) + 1)

    axes[0].plot(
        waypoint_indices,
        linear["pos_errors"] * 100.0,
        marker="o",
        label="linear waypoint error",
    )
    axes[0].plot(
        waypoint_indices,
        catmull["pos_errors"] * 100.0,
        marker="o",
        label="catmull_rom waypoint error",
    )
    axes[0].set_xlabel("waypoint index")
    axes[0].set_ylabel("position error [cm]")
    axes[0].set_title("Waypoint tracking error")
    axes[0].grid(True)
    axes[0].legend()

    axes[1].plot(
        waypoint_positions[:, 0],
        waypoint_positions[:, 2],
        "k--o",
        label="original waypoint linear connection",
    )
    axes[1].plot(
        linear_x,
        linear_z,
        label="actual TCP linear",
    )
    axes[1].plot(
        catmull_x,
        catmull_z,
        label="actual TCP catmull_rom",
    )
    axes[1].set_xlabel("x [m]")
    axes[1].set_ylabel("z [m]")
    axes[1].set_title("X-Z TCP path")
    axes[1].axis("equal")
    axes[1].grid(True)
    axes[1].legend()

    fig.tight_layout()
    fig.savefig(PLOT_PATH, dpi=160)
    print(f"\nSaved plot: {PLOT_PATH}")


def main():
    robot_model = RobotModel()

    client = RemoteControllerClient(
        SERVER_URL,
        capacity=BUFFER_CAPACITY,
        horizon_prev=1,
        sensor_size=22,
        default_frame_name="panda_hand_tcp",
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

        stiffness = make_cartesian_stiffness()

        client.empty_buffer()
        state = client.wait_for_first_udp(timeout=2.0)
        q0 = state["q"]

        T_current = client.get_tcp_pose_from_q(robot_model, q0)
        T_fixed_start = make_fixed_start_pose(client, robot_model)

        print("\nUnit check:")
        print("all Cartesian positions are meters.")
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

        results = {}

        for mode in MODES:
            results[mode] = run_mode(
                client,
                robot_model,
                mode,
                T_fixed_start,
                stiffness,
            )

            time.sleep(0.5)

        print("\nComparison:")
        for mode in MODES:
            pos_errors = results[mode]["pos_errors"]
            print(
                f"{mode:12s} "
                f"mean_pos_error={pos_errors.mean():.5f} m, "
                f"max_pos_error={pos_errors.max():.5f} m, "
                f"measured_hz={results[mode]['measured_hz']:.1f}"
            )

        plot_results(results)

    finally:
        client.close()


if __name__ == "__main__":
    main()
