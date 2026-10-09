from remote_controller.RemoteControllerClient import RemoteControllerClient
import math
import time


SERVER_URL = "http://localhost:8008/RPC2"
UDP_IP = "127.0.0.1"
UDP_PORT = 9000

UDP_FREQUENCY_HZ = 500
BUFFER_CAPACITY = 20000

ENABLE_IDLE_POLL = False
IDLE_POLL_HZ = 1

SAMPLES_PER_SEGMENT = 100
SLOWDOWN_FACTOR = 1.0
PATH_MODE = "catmull_rom"

WAYPOINT_REACHED_TOL = 0.035
WAIT_TIMEOUT_S = 15.0


def make_stiffness():
    return [
        [200.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 200.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 200.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 200.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 50.0],
    ]


def q_distance(q1, q2):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(q1, q2)))


def make_waypoints(q0):
    waypoints = []

    for i in range(1, 16):
        phase = i / 15.0

        q = q0.copy()
        q[0] += 0.08 * math.sin(math.pi * phase)
        q[3] -= 0.06 * math.sin(math.pi * phase)
        q[5] += 0.04 * math.sin(0.5 * math.pi * phase)
        q[6] += 0.03 * phase

        waypoints.append(q)

    return waypoints


def get_udp_frames(client):
    buffer = client.udp_receiver.data_buffer
    with buffer.lock:
        return list(buffer.buffer)


def analyze_waypoints(frames, waypoints):
    

    if not frames:
        print("no UDP frames captured")
        return

    qs = [frame.data[:7] for frame in frames]
    timestamps = [frame.timestamp for frame in frames]
    t0 = timestamps[0]

    print("\nWaypoint check from UDP frames:")

    previous_best_index = 0

    for waypoint_index, waypoint in enumerate(waypoints):
        best_index = None
        best_distance = float("inf")

        for i in range(previous_best_index, len(qs)):
            dist = q_distance(qs[i], waypoint)
            if dist < best_distance:
                best_distance = dist
                best_index = i

        reached = best_distance <= WAYPOINT_REACHED_TOL
        best_time = timestamps[best_index] - t0 if best_index is not None else None

        if best_index is not None:
            previous_best_index = best_index

        print(
            f"waypoint {waypoint_index + 1:02d}: "
            f"closest_distance={best_distance:.5f}, "
            f"closest_time={best_time:.3f}s, "
            f"reached={reached}"
        )


def main():
    client = RemoteControllerClient(
        SERVER_URL,
        capacity=BUFFER_CAPACITY,
        horizon_prev=1,
        sensor_size=22,
    )

    try:
        result = client.init(
            udp_ip=UDP_IP,
            udp_port=UDP_PORT,
            udp_frequency_hz=UDP_FREQUENCY_HZ,
            recover_before_init=True,
        )
        print(f"init result: {result} ({client.decode_rpc_result(result)})")

        if ENABLE_IDLE_POLL:
            result = client.set_idle_state_poll_frequency(IDLE_POLL_HZ)
            print(
                f"set idle poll result: "
                f"{result} ({client.decode_rpc_result(result)})"
            )

        state = client.wait_for_first_udp(timeout=2.0)
        q0 = state["q"]

        waypoints = make_waypoints(q0)
        stiffness = make_stiffness()

        vmax = [0.35] * 7
        acc_max = [1.0] * 7

        client.empty_buffer()
        client.set_slowdown_factor(SLOWDOWN_FACTOR)
        start_time = time.monotonic()

        result = client.movej(
            waypoints,
            stiffness=stiffness,
            dq_max=vmax,
            ddq_max=acc_max,
            queue=False,
            samples_per_segment=SAMPLES_PER_SEGMENT,
            path_mode=PATH_MODE,
        )
        print(f"moveJ path result: {result} ({client.decode_rpc_result(result)})")

        while True:
            arm_state = client.get_arm_state()
            elapsed = time.monotonic() - start_time

            if arm_state != "MOVING":
                break

            if elapsed > WAIT_TIMEOUT_S:
                print(f"timeout waiting for motion done, arm_state={arm_state}")
                break

            time.sleep(0.02)

        end_time = time.monotonic()
        elapsed = end_time - start_time

        frames = get_udp_frames(client)
        packet_count = len(frames)
        measured_hz = packet_count / elapsed if elapsed > 0.0 else 0.0
        expected_packets = UDP_FREQUENCY_HZ * elapsed

        print("\nUDP moving path result:")
        print(f"packet_count: {packet_count}")
        print(f"elapsed_s: {elapsed:.3f}")
        print(f"target_hz: {UDP_FREQUENCY_HZ}")
        print(f"measured_hz: {measured_hz:.1f}")
        print(f"expected_packets: {expected_packets:.1f}")
        print(f"final arm state: {client.get_arm_state()}")

        analyze_waypoints(frames, waypoints)

    finally:
        client.close()


if __name__ == "__main__":
    main()
