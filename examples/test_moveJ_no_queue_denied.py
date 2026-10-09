from remote_controller.RemoteControllerClient import RemoteControllerClient

import math
import threading
import time


SERVER_URL = "http://localhost:8008/RPC2"
UDP_IP = "127.0.0.1"
UDP_PORT = 9000
UDP_FREQUENCY_HZ = 500
BUFFER_CAPACITY = 30000

EXPECTED_DENIED_MOVING = -2
WAIT_MOVING_TIMEOUT_S = 3.0
WAIT_FINISH_TIMEOUT_S = 30.0

JOINT_LOWER = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
JOINT_UPPER = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]


def make_joint_stiffness():
    return [
        [200.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 200.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 200.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 200.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 50.0],
    ]


def safe_joint_value(current, desired, joint_index, margin=0.12):
    lo = JOINT_LOWER[joint_index] + margin
    hi = JOINT_UPPER[joint_index] - margin

    if current < lo or current > hi:
        return current

    return min(max(desired, lo), hi)


def q_distance(q1, q2):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(q1, q2)))


def make_movej_target(q0, direction=1.0):
    q = list(q0)

    deltas = {
        0: 0.14 * direction,
        3: -0.10 * direction,
        5: 0.08 * direction,
        6: 0.06 * direction,
    }

    for joint_index, delta in deltas.items():
        q[joint_index] = safe_joint_value(
            q0[joint_index],
            q0[joint_index] + delta,
            joint_index,
        )

    if q_distance(q0, q) < 0.03:
        raise RuntimeError("Generated moveJ target is too close to q0.")

    return q


def ensure_ok(client, result, name):
    client.print_rpc_result(result, name)
    if result != 0:
        raise RuntimeError(
            f"{name} failed: {result} ({client.decode_rpc_result(result)})"
        )


def run_first_movej(worker_result, q_target, stiffness, dq_max, ddq_max):
    client = RemoteControllerClient(SERVER_URL)

    try:
        result = client.movej(
            q_target,
            stiffness=stiffness,
            dq_max=dq_max,
            ddq_max=ddq_max,
            queue=False,
        )
        worker_result["result"] = result
    except Exception as exc:
        worker_result["exception"] = exc


def wait_until_arm_moving_started(client, worker_result, timeout):
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        if "exception" in worker_result:
            raise RuntimeError("First moveJ raised before MOVING.") from worker_result["exception"]

        if "result" in worker_result:
            raise RuntimeError(
                "First moveJ returned before arm entered MOVING. "
                "Increase target distance or reduce dq_max."
            )

        arm_state = client.get_arm_state()
        if arm_state == "MOVING":
            return

        if arm_state == "ERROR":
            raise RuntimeError("Arm entered ERROR while waiting for MOVING.")

        time.sleep(0.02)

    raise TimeoutError("Timed out waiting for arm to enter MOVING.")


def main():
    state_client = RemoteControllerClient(
        SERVER_URL,
        capacity=BUFFER_CAPACITY,
        horizon_prev=1,
        sensor_size=22,
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

        state = state_client.wait_for_first_udp(timeout=2.0)
        q0 = state["q"]

        first_target = make_movej_target(q0, direction=1.0)
        denied_target = make_movej_target(q0, direction=-1.0)

        stiffness = make_joint_stiffness()
        dq_max = [0.04] * 7
        ddq_max = [0.08] * 7

        print("q0:")
        print([round(x, 5) for x in q0])
        print("first no-queue moveJ target:")
        print([round(x, 5) for x in first_target])

        print("\nStart first moveJ without queue in background")
        worker = threading.Thread(
            target=run_first_movej,
            args=(worker_result, first_target, stiffness, dq_max, ddq_max),
            daemon=True,
        )
        worker.start()

        wait_until_arm_moving_started(
            state_client,
            worker_result,
            timeout=WAIT_MOVING_TIMEOUT_S,
        )
        print("arm state before second command:", state_client.get_arm_state())

        print("\nSend second moveJ without queue while first moveJ is moving")
        denied_result = probe_client.movej(
            denied_target,
            stiffness=stiffness,
            dq_max=dq_max,
            ddq_max=ddq_max,
            queue=False,
        )
        probe_client.print_rpc_result(denied_result, "second no-queue moveJ")

        if denied_result != EXPECTED_DENIED_MOVING:
            state_client.stop_arm_motion()
            raise RuntimeError(
                "Expected second no-queue moveJ to be denied while moving. "
                f"Got {denied_result} ({state_client.decode_rpc_result(denied_result)})."
            )

        print("PASS: second no-queue moveJ was denied while arm was moving.")

        arm_state = state_client.wait_until_arm_idle_ok(timeout=WAIT_FINISH_TIMEOUT_S)
        print("arm state after first moveJ:", arm_state)

        worker.join(timeout=3.0)
        if worker.is_alive():
            state_client.stop_arm_motion()
            raise RuntimeError("First moveJ RPC did not return after arm became idle.")

        if "exception" in worker_result:
            raise RuntimeError("First moveJ raised.") from worker_result["exception"]

        ensure_ok(state_client, worker_result["result"], "first moveJ")

        print("\nTest finished successfully.")

    finally:
        if worker is not None and worker.is_alive():
            state_client.stop_arm_motion()

        state_client.close()
        probe_client.close()


if __name__ == "__main__":
    main()
