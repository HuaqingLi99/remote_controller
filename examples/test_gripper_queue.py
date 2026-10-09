from remote_controller import RemoteControllerClient

import threading
import time


SERVER_URL = "http://localhost:8008/RPC2"
UDP_IP = "127.0.0.1"
UDP_PORT = 9000
UDP_FREQUENCY_HZ = 500

WIDTH_1 = 0.05
WIDTH_2 = 0.04
SPEED = 0.10
FORCE = 10.0
EPSILON_INNER = 0.005
EPSILON_OUTER = 0.005

WAIT_TIMEOUT_S = 10.0


def first_command():
    client = RemoteControllerClient(SERVER_URL)

    result = client.grasp(
        WIDTH_1,
        SPEED,
        FORCE,
        EPSILON_INNER,
        EPSILON_OUTER,
        queue=True,
    )
    client.print_rpc_result(result, "first queue grasp")


def second_command():
    time.sleep(0.05)

    client = RemoteControllerClient(SERVER_URL)

    result = client.grasp(
        WIDTH_2,
        SPEED,
        FORCE,
        EPSILON_INNER,
        EPSILON_OUTER,
        queue=True,
    )
    client.print_rpc_result(result, "second queue grasp")


def main():
    init_client = RemoteControllerClient(SERVER_URL)

    try:
        init_result = init_client.init(
            udp_ip=UDP_IP,
            udp_port=UDP_PORT,
            udp_frequency_hz=UDP_FREQUENCY_HZ,
            recover_before_init=True,
        )
        init_client.print_rpc_result(init_result, "init")

        t1 = threading.Thread(target=first_command)
        t2 = threading.Thread(target=second_command)

        t1.start()
        t2.start()

        t1.join()
        t2.join()

        final_state = init_client.wait_until_gripper_moving_finished(
            timeout=WAIT_TIMEOUT_S,
        )

        print(f"final gripper state: {final_state}")

    finally:
        init_client.close()


if __name__ == "__main__":
    main()
