from remote_controller.RemoteControllerClient import RemoteControllerClient
from remote_controller.robot_kinematics import RobotModel


SERVER_URL = "http://localhost:8008/RPC2"
UDP_IP = "127.0.0.1"
UDP_PORT = 9000
UDP_FREQUENCY_HZ = 500


def make_cartesian_stiffness():
    return [
        [150.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 150.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 150.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 10.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 10.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 10.0],
    ]


def main():
    robot_model = RobotModel()

    client = RemoteControllerClient(
        SERVER_URL,
        capacity=5000,
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
        client.print_rpc_result(result, "init")

        stiffness = make_cartesian_stiffness()

        # Move down along world Z by 3 cm.
        # Unit is meter: -0.03 m = -3 cm.
        result = client.movecart_relative(
            robot_model=robot_model,
            dx=0.0,
            dy=0.0,
            dz=-0.1,
            droll=0.0,
            dpitch=0.0,
            dyaw=0.0,
            stiffness=stiffness,
            vmax=0.03,
            acc_max=0.05,
            nullspace_stiffness=0.0,
            queue=False,
            reference_frame="world",
        )
        client.print_rpc_result(result, "moveC relative down z 3cm")

    finally:
        client.close()


if __name__ == "__main__":
    main()
