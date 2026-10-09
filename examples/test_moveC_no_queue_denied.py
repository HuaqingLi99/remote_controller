from remote_controller import RemoteControllerClient, RobotModel

import math
import threading
import time

import numpy as np


SERVER_URL = "http://localhost:8008/RPC2"
UDP_IP = "127.0.0.1"
UDP_PORT = 9000
UDP_FREQUENCY_HZ = 500
BUFFER_CAPACITY = 30000

EXPECTED_DENIED_MOVING = -2
WAIT_MOVING_TIMEOUT_S = 3.0
WAIT_FINISH_TIMEOUT_S = 30.0

FIRST_DZ_M = 0.02
DENIED_DX_M = 0.02
DENIED_DZ_M = 0.02


def make_cartesian_stiffness():
    return [
        [1500.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 1500.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1500.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1500.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 1500.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 1500.0],
    ]


def flatten_pose(T):
    return [float(x) for x in T.reshape(-1)]


def make_relative_pose(T_start, dx_m=0.0, dy_m=0.0, dz_m=0.0):
    T = np.array(T_start, dtype=float).copy()
    T[0, 3] += dx_m
    T[1, 3] += dy_m
    T[2, 3] += dz_m
    return T


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


def ensure_ok(client, result, name):
    client.print_rpc_result(result, name)
    if result != 0:
        raise RuntimeError(
            f"{name} failed: {result} ({client.decode_rpc_result(result)})"
        )


def run_first_movec(worker_result, T_target_flat, stiffness):
    client = RemoteControllerClient(SERVER_URL)

    try:
        result = client.movecart(
            T_target_flat,
            stiffness=stiffness,
            vmax_linear=0.01,
            acc_max_linear=0.02,
            vmax_angular=0.10,
            acc_max_angular=0.02,
            nullspace_stiffness=0.0,
            queue=False,
        )
        worker_result["result"] = result
    except Exception as exc:
        worker_result["exception"] = exc


def wait_until_arm_moving_started(client, worker_result, timeout):
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        if "exception" in worker_result:
            raise RuntimeError("First moveC raised before MOVING.") from worker_result["exception"]

        if "result" in worker_result:
            raise RuntimeError(
                "First moveC returned before arm entered MOVING. "
                "Increase target distance or reduce vmax_linear."
            )

        arm_state = client.get_arm_state()
        if arm_state == "MOVING":
            return

        if arm_state == "ERROR":
            raise RuntimeError("Arm entered ERROR while waiting for MOVING.")

        time.sleep(0.02)

    raise TimeoutError("Timed out waiting for arm to enter MOVING.")


def main():
    robot_model = RobotModel()

    state_client = RemoteControllerClient(
        SERVER_URL,
        capacity=BUFFER_CAPACITY,
        horizon_prev=1,
        sensor_size=22,
        default_frame_name="panda_hand_tcp",
    )
    probe_client = RemoteControllerClient(SERVER_URL)

    worker_result = {}
    worker = None

    try:
        result = state_client.init(
            udp_ip=UDP_IP,
            udp_port=UDP_PORT,
            udp_frequency_hz=UDP_FREQUENCY_HZ,
            recover_before_init=True,
        )
        ensure_ok(state_client, result, "init")

        stiffness = make_cartesian_stiffness()

        T0 = state_client.get_current_tcp_pose(robot_model, flush=True)
        first_target_T = make_relative_pose(T0, dz_m=FIRST_DZ_M)
        denied_target_T = make_relative_pose(
            T0,
            dx_m=DENIED_DX_M,
            dz_m=DENIED_DZ_M,
        )

        print_pose_translation("start TCP", T0)
        print_pose_translation("first no-queue moveC target", first_target_T)

        print("\nStart first moveC without queue in background")
        worker = threading.Thread(
            target=run_first_movec,
            args=(worker_result, flatten_pose(first_target_T), stiffness),
            daemon=True,
        )
        worker.start()

        wait_until_arm_moving_started(
            state_client,
            worker_result,
            timeout=WAIT_MOVING_TIMEOUT_S,
        )
        print("arm state before second command:", state_client.get_arm_state())

        print("\nSend second moveC without queue while first moveC is moving")
        denied_result = probe_client.movecart(
            flatten_pose(denied_target_T),
            stiffness=stiffness,
            vmax_linear=0.01,
            acc_max_linear=0.02,
            vmax_angular=0.10,
            acc_max_angular=0.20,
            nullspace_stiffness=0.0,
            queue=False,
        )
        probe_client.print_rpc_result(denied_result, "second no-queue moveC")

        if denied_result != EXPECTED_DENIED_MOVING:
            state_client.stop_arm_motion()
            raise RuntimeError(
                "Expected second no-queue moveC to be denied while moving. "
                f"Got {denied_result} ({state_client.decode_rpc_result(denied_result)})."
            )

        print("PASS: second no-queue moveC was denied while arm was moving.")

        arm_state = state_client.wait_until_arm_idle_ok(timeout=WAIT_FINISH_TIMEOUT_S)
        print("arm state after first moveC:", arm_state)

        worker.join(timeout=3.0)
        if worker.is_alive():
            state_client.stop_arm_motion()
            raise RuntimeError("First moveC RPC did not return after arm became idle.")

        if "exception" in worker_result:
            raise RuntimeError("First moveC raised.") from worker_result["exception"]

        ensure_ok(state_client, worker_result["result"], "first moveC")

        T_after = state_client.get_current_tcp_pose(robot_model, flush=True)
        print_pose_translation("TCP after first moveC", T_after)

        pos_err_m, rot_err_rad = pose_error(T_after, first_target_T)
        print(
            "first moveC final check: "
            f"pos_error={pos_err_m:.5f} m, "
            f"rot_error={rot_err_rad:.5f} rad"
        )

        print("\nTest finished successfully.")

    finally:
        if worker is not None and worker.is_alive():
            state_client.stop_arm_motion()

        state_client.close()
        probe_client.close()


if __name__ == "__main__":
    main()
