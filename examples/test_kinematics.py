from remote_controller.RemoteControllerClient import RemoteControllerClient
from remote_controller.robot_kinematics import RobotModel

import math
import time


SERVER_URL = "http://localhost:8008/RPC2"
UDP_IP = "127.0.0.1"
UDP_PORT = 9000
UDP_FREQUENCY_HZ = 500
BUFFER_CAPACITY = 5000

FRAME_NAME = "panda_hand_tcp"

WAIT_AFTER_MOVEC_S = 0.5


def make_cartesian_stiffness():
    return [
        [50.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 50.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 50.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 6.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 6.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 6.0],
    ]


def flatten_pose(T):
    return [float(x) for x in T.reshape(-1)]


def q_distance(q1, q2):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(q1, q2)))


def q_abs_max_error(q1, q2):
    return max(abs(a - b) for a, b in zip(q1, q2))


def print_q(label, q):
    print(label)
    print([round(float(x), 6) for x in q])


def main():
    robot_model = RobotModel()

    print("RobotModel check:")
    print("nq:", robot_model.nq)
    print("nv:", robot_model.nv)
    print("has panda_hand:", "panda_hand" in robot_model.list_frames())
    print("has panda_hand_tcp:", "panda_hand_tcp" in robot_model.list_frames())

    if robot_model.nq != 7 or robot_model.nv != 7:
        raise RuntimeError("RobotModel should be 7DOF for this UDP q test.")

    if FRAME_NAME not in robot_model.list_frames():
        raise RuntimeError(f"{FRAME_NAME} not found in URDF.")

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
        print(f"\ninit result: {result} ({client.decode_rpc_result(result)})")

        client.empty_buffer()
        state0 = client.wait_for_first_udp(timeout=2.0)
        q0 = state0["q"]

        print_q("\nq0 from UDP:", q0)

        pose = robot_model.get_frame_pose(q0, frame_name=FRAME_NAME)
        T = pose["T"]

        print(f"\nFK pose from q0 using frame '{FRAME_NAME}':")
        print("translation:", [round(float(x), 6) for x in T[:3, 3]])

        stiffness = make_cartesian_stiffness()

        result = client.movecart(
            flatten_pose(T),
            stiffness=stiffness,
            vmax_linear=0.03,
            acc_max_linear=0.05,
            vmax_angular=0.20,
            acc_max_angular=0.40,
            nullspace_stiffness=1.0,
            queue=False,
        )
        print(f"\nmoveC same-pose result: {result} ({client.decode_rpc_result(result)})")

        time.sleep(WAIT_AFTER_MOVEC_S)

        client.empty_buffer()
        state1 = client.wait_for_first_udp(timeout=2.0)
        q1 = state1["q"]

        print_q("\nq1 from UDP after moveC:", q1)

        norm_error = q_distance(q0, q1)
        max_error = q_abs_max_error(q0, q1)

        print("\nq comparison:")
        print(f"q_norm_error: {norm_error:.8f} rad")
        print(f"q_max_abs_error: {max_error:.8f} rad")

        if norm_error < 0.01 and max_error < 0.005:
            print("\nPASS: FK frame is probably close to Franka O_T_EE.")
        else:
            print("\nWARNING: q changed noticeably.")
            print("This means the URDF frame may not match Franka O_T_EE exactly,")
            print("or moveC did not settle exactly at the sent pose.")

    finally:
        client.close()


if __name__ == "__main__":
    main()
