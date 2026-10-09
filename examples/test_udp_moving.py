from remote_controller.RemoteControllerClient import RemoteControllerClient
import time


SERVER_URL = "http://localhost:8008/RPC2"
UDP_IP = "127.0.0.1"
UDP_PORT = 9000
UDP_FREQUENCY_HZ = 500
TEST_DURATION_S = 5.0
BUFFER_CAPACITY = 5000


IDLE_POLL_HZ = 1


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


def make_queue_targets(q0, count=10):
    targets = []

    for i in range(count):
        q = q0.copy()

        sign = 1.0 if i % 2 == 0 else -1.0
        q[0] += sign * 0.04
        q[3] -= sign * 0.03
        q[5] += sign * 0.02

        targets.append(q)

    return targets


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

        client.set_idle_state_poll_frequency(1)
        state = client.wait_for_first_udp(timeout=2.0)
        q0 = state["q"]

        stiffness = make_stiffness()
        vmax = [0.20] * 7
        acc_max = [0.20] * 7
        targets = make_queue_targets(q0, count=10)

        for index, q_target in enumerate(targets):
            result = client.movej(
                q_target,
                stiffness=stiffness,
                vmax=vmax,
                acc_max=acc_max,
                queue=True,
            )
            print(
                f"queued moveJ {index + 1}/{len(targets)}: "
                f"{result} ({client.decode_rpc_result(result)})"
            )

        client.empty_buffer()
        start_time = time.monotonic()

        print(
            f"\nstart UDP moving test: target={UDP_FREQUENCY_HZ} Hz, "
            f"duration={TEST_DURATION_S}s, capacity={BUFFER_CAPACITY}"
        )

        time.sleep(TEST_DURATION_S)

        end_time = time.monotonic()
        elapsed = end_time - start_time

        packet_count = client.udp_receiver.data_buffer.get_size()
        measured_hz = packet_count / elapsed if elapsed > 0.0 else 0.0
        expected_packets = UDP_FREQUENCY_HZ * elapsed

        print("\nUDP moving result:")
        print(f"packet_count: {packet_count}")
        print(f"elapsed_s: {elapsed:.3f}")
        print(f"target_hz: {UDP_FREQUENCY_HZ}")
        print(f"measured_hz: {measured_hz:.1f}")
        print(f"expected_packets: {expected_packets:.1f}")
        print(f"final arm state: {client.get_arm_state()}")

    finally:
        client.close()


if __name__ == "__main__":
    main()
