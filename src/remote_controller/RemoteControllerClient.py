import time
import xmlrpc.client

import numpy as np
from .trackj_streamer import TrackC2Streamer, TrackCStreamer, TrackJ2Streamer, TrackJStreamer
from .udp_receiver import UDPReceiver


class RemoteControllerClient:
    """XML-RPC and UDP client for arm, gripper, and state feedback.

    Public methods return the server RPC result code unless the docstring says
    they return decoded state, UDP data, or kinematic quantities.
    """

    def __init__(
        self,
        server_url: str,
        capacity: int = 16,
        horizon_prev: int = 7,
        sensor_size: int = 22,
        default_frame_name="panda_hand_tcp"
    ):
        """Create the client and XML-RPC proxy without starting UDP yet.

        Args:
            server_url: XML-RPC endpoint, for example
                "http://localhost:8008/RPC2".
            capacity: Number of UDP frames kept in the local DataBuffer.
            horizon_prev: Number of recent frames returned by get_padded_data().
            sensor_size: Number of values used from each UDP frame.
            default_frame_name: TCP frame used by helper FK methods.

        The default UDP buffer is intentionally small for real-time policy/VLA
        inference. Use get_padded_data() for a fixed-size history window, and
        increase capacity only for logging, plotting, or long motion analysis.
        """


        self.server_url = server_url
        self.proxy = xmlrpc.client.ServerProxy(server_url, allow_none=True)

        self.udp_receiver = None
        self.initialized = False

        self.capacity = capacity
        self.horizon_prev = horizon_prev
        self.sensor_size = sensor_size
        self.udp_frequency_hz = None
        self.default_frame_name = default_frame_name
        self._last_used_seq = -1

    def init(
        self,
        udp_ip: str = "127.0.0.1",
        udp_port: int = 9000,
        udp_frequency_hz: int = 1000,
        lower_torque=None,
        upper_torque=None,
        lower_force=None,
        upper_force=None,
        recover_before_init=True,
    ):
        """Start local UDP receiving and initialize the server session.

        Args:
            udp_ip: Local IP address where this client receives UDP state.
            udp_port: Local UDP port for robot state streaming.
            udp_frequency_hz: Requested server UDP publish frequency in Hz.
            lower_torque: Optional 7-element lower torque threshold.
            upper_torque: Optional 7-element upper torque threshold.
            lower_force: Optional 6-element lower Cartesian force threshold.
            upper_force: Optional 6-element upper Cartesian force threshold.
            recover_before_init: If True, call recover_system() before init.

        Returns:
            RPC result code. 0 means OK.
        """

        if lower_torque is None:
            lower_torque = [20.0] * 7
        if upper_torque is None:
            upper_torque = [20.0] * 7
        if lower_force is None:
            lower_force = [10.0] * 6
        if upper_force is None:
            upper_force = [10.0] * 6
            
        if recover_before_init:
            recover_result = self.recover_system()
            if recover_result != 0:
                raise RuntimeError(
                    f"recover_system failed before init: "
                    f"{recover_result} ({self.decode_rpc_result(recover_result)})"
                )


        if self.udp_receiver is not None:
            self.udp_receiver.close()

        self.udp_receiver = UDPReceiver(
            udp_ip=udp_ip,
            udp_port=udp_port,
            capacity=self.capacity,
            horizon_prev=self.horizon_prev,
            sensor_size=self.sensor_size,
        )
        self.udp_receiver.start_receiving(timeout=0.1)

        try:
            result = self.proxy.initSession(
                udp_ip,
                udp_port,
                udp_frequency_hz,
                lower_torque,
                upper_torque,
                lower_force,
                upper_force,
            )
        except Exception:
            self.udp_receiver.close()
            self.udp_receiver = None
            raise

        self.udp_receiver.flush()
        self.udp_frequency_hz = udp_frequency_hz
        self.initialized = True
        self._last_used_seq = -1
        return result


    def get_latest_state(self, allow_stale: bool = True):
        """Return the newest UDP state and freshness metadata.

        Args:
            allow_stale: If False, stale UDP frames are treated as unavailable.

        Returns:
            (state, info). state is None when no usable UDP frame is available.
            state contains q, dq, K_F_ext_hat, arm_state, and gripper_state.
            info contains age, level, and is_new.
        """

        if not self.initialized or self.udp_receiver is None:
            raise RuntimeError("Client is not initialized")

        state, age, level, seq = self.udp_receiver.get_latest_frame_info()

        if level == "empty":
            return None, {
                "is_new": False,
                "age": None,
                "level": "empty",
            }

        if level in ("expired", "too_old"):
            return None, {
                "is_new": False,
                "age": age,
                "level": level,
            }

        if level == "stale" and not allow_stale:
            return None, {
                "is_new": False,
                "age": age,
                "level": level,
            }

        is_new = seq != self._last_used_seq
        self._last_used_seq = seq

        return state, {
            "is_new": is_new,
            "age": age,
            "level": level,
        }

    def get_padded_data(self, allow_stale: bool = True):
        """Return a padded UDP history window and metadata.

        This is the recommended state read for VLA/policy inference because it
        uses DataBuffer and always returns a stable history shape.

        Args:
            allow_stale: If False, stale UDP frames are treated as unavailable.

        Returns:
            (data, meta). data has one flattened history row with
            horizon_prev * sensor_size values, or [] when unavailable.
        """

        if not self.initialized or self.udp_receiver is None:
            raise RuntimeError("Client is not initialized")

        data, meta = self.udp_receiver.get_padded_data_with_info(
            last_seen_seq=self._last_used_seq
        )

        if meta["level"] == "empty":
            return [], meta

        if meta["level"] in ("expired", "too_old"):
            return [], meta

        if meta["level"] == "stale" and not allow_stale:
            return [], meta

        self._last_used_seq = meta["latest_seq"]
        return data, meta

    def wait_for_first_udp(self, timeout=None):
        """Block until a usable UDP frame is available, then return it.

        Args:
            timeout: Maximum wait time in seconds, or None to wait forever.

        Returns:
            Latest UDP state dict with q, dq, K_F_ext_hat, arm_state, and
            gripper_state.
        """

        if not self.initialized or self.udp_receiver is None:
            raise RuntimeError("Client is not initialized")

        deadline = None if timeout is None else time.monotonic() + timeout

        while True:
            state, info = self.get_latest_state(allow_stale=True)
            if state is not None:
                return state

            if deadline is not None and time.monotonic() >= deadline:
                raise TimeoutError(f"wait_for_first_udp timed out after {timeout} seconds")

            time.sleep(0.01)

    def empty_buffer(self):
        """Flush the local UDP buffer and reset the sequence tracker."""

        if not self.initialized or self.udp_receiver is None:
            raise RuntimeError("Client is not initialized")

        self.udp_receiver.flush()
        self._last_used_seq = -1

    def get_udp_period(self):
        """Return the target UDP period in seconds, or None before init."""

        if self.udp_frequency_hz is None:
            return None
        return 1.0 / self.udp_frequency_hz

    def close(self):
        """Close the UDP receiver and reset local client session state."""

        if self.udp_receiver is not None:
            self.udp_receiver.close()
            self.udp_receiver = None
        self.initialized = False
        self.udp_frequency_hz = None
        self._last_used_seq = -1



    def movej_no_queue(self, q_goal, stiffness, vmax, acc_max):
        """Execute a direct single-target moveJ RPC call.

        Args:
            q_goal: 7-element joint target in radians.
            stiffness: 7-element joint stiffness list.
            vmax: 7-element joint velocity limit in rad/s.
            acc_max: 7-element joint acceleration limit in rad/s^2.
        """

        return self.proxy.moveJ(q_goal, stiffness, vmax, acc_max)
    




    def movej_queue(self, q_goal, stiffness, vmax, acc_max):
        """Queue a single-target moveJ command and return its accept code.

        Args:
            q_goal: 7-element joint target in radians.
            stiffness: 7-element joint stiffness list.
            vmax: 7-element joint velocity limit in rad/s.
            acc_max: 7-element joint acceleration limit in rad/s^2.
        """

        return self.proxy.moveJ_queue(q_goal, stiffness, vmax, acc_max)
    
    


       
    def movej_path_no_queue(
        self,
        waypoints,
        samples_per_segment,
        path_mode,
        stiffness,
        dq_max,
        ddq_max=None,
    ):
        """Execute a direct moveJPath RPC call for joint waypoints.

        Args:
            waypoints: List of 7-element joint targets in radians.
            samples_per_segment: Number of base replay samples per segment.
            path_mode: Path interpolation mode, such as "linear".
            stiffness: 7-element joint stiffness list.
            dq_max: 7-element joint velocity limit in rad/s.
            ddq_max: Ignored by moveJPath; kept for API compatibility.
        """

        if ddq_max is not None:
            print(
                "Warning: ddq_max is ignored by moveJPath. "
                "Path replay only checks dq_max."
            )
        else:
            ddq_max = [0.0] * 7

        return self.proxy.moveJPath(
            waypoints,
            samples_per_segment,
            path_mode,
            stiffness,
            dq_max,
            ddq_max,
        )



    def movej_path_queue(
        self,
        waypoints,
        samples_per_segment,
        path_mode,
        stiffness,
        dq_max,
        ddq_max=None,
    ):
        """Queue a moveJPath command for joint waypoints.

        Args:
            waypoints: List of 7-element joint targets in radians.
            samples_per_segment: Number of base replay samples per segment.
            path_mode: Path interpolation mode, such as "linear".
            stiffness: 7-element joint stiffness list.
            dq_max: 7-element joint velocity limit in rad/s.
            ddq_max: Ignored by moveJPath_queue; kept for API compatibility.
        """

        if ddq_max is not None:
            print(
                "Warning: ddq_max is ignored by moveJPath_queue. "
                "Path replay only checks dq_max."
            )
        else:
            ddq_max = [0.0] * 7

        return self.proxy.moveJPath_queue(
            waypoints,
            samples_per_segment,
            path_mode,
            stiffness,
            dq_max,
            ddq_max,
        )

    def movej(
        self,
        target,
        stiffness,
        dq_max,
        ddq_max=None,
        queue=False,
        samples_per_segment=10,
        path_mode="linear",
    ):
        """Dispatch moveJ to single-target or waypoint-path RPC.

        Args:
            target: Either a 7-element joint target or an Nx7 waypoint list.
            stiffness: Joint stiffness list.
            dq_max: Joint velocity limit in rad/s.
            ddq_max: Required for a single target; ignored for waypoint paths.
            queue: If True, enqueue instead of executing immediately.
            samples_per_segment: Base replay samples per path segment.
            path_mode: Path interpolation mode, such as "linear".
        """

        if not isinstance(target, (list, tuple)):
            raise TypeError("target must be a list or tuple")

        # Case 1: single joint target, shape [7]
        if len(target) == 7 and all(isinstance(x, (int, float)) for x in target):
            if ddq_max is None:
                raise ValueError("ddq_max is required for single moveJ.")

            if queue:
                return self.movej_queue(target, stiffness, dq_max, ddq_max)
            return self.movej_no_queue(target, stiffness, dq_max, ddq_max)


        # Case 2: waypoint path, shape [N][7]
        if len(target) > 0 and all(isinstance(row, (list, tuple)) and len(row) == 7 for row in target):
            if queue:
                return self.movej_path_queue(
                    target,
                    samples_per_segment,
                    path_mode,
                    stiffness,
                    dq_max,
                    ddq_max,
                )
            return self.movej_path_no_queue(
                target,
                samples_per_segment,    
                path_mode,
                stiffness,
                dq_max,
                ddq_max,
            )

        raise ValueError("target must be either a 7-element joint vector or an Nx7 waypoint list")


    def create_trackj_streamer(
        self,
        command_ip="127.0.0.1",
        command_port=9100,
        stream_hz=500,
        samples_per_segment=50,
    ):
        """Create a UDP joint-reference streamer for trackJ.

        trackJ is intended for VLA/policy-style streaming control. The server is
        started and stopped through XML-RPC, while q_ref samples are streamed over
        a dedicated UDP command port.

        Args:
            command_ip: Server IP address that receives trackJ UDP q_ref packets.
            command_port: Server UDP port for trackJ commands.
            stream_hz: UDP command streaming frequency in Hz.
            samples_per_segment: Number of interpolated samples per VLA action
                segment.

        Returns:
            TrackJStreamer instance. Use start(), update_waypoints(), finish(),
            stop(), and close() on it.
        """

        if not self.initialized:
            raise RuntimeError("Client is not initialized")

        return TrackJStreamer(
            client=self,
            command_ip=command_ip,
            command_port=command_port,
            stream_hz=stream_hz,
            samples_per_segment=samples_per_segment,
        )

    def create_trackj2_streamer(
        self,
        command_ip="127.0.0.1",
        command_port=9110,
        stream_hz=100,
        action_gap_s=0.01,
    ):
        """Create an async UDP joint-target streamer for trackJ2.

        trackJ2 sends raw q_target samples over UDP. The client owns chunk
        timing, while the server smooths the latest target with a second-order
        dynamic filter before joint impedance tracking.
        """

        if not self.initialized:
            raise RuntimeError("Client is not initialized")

        return TrackJ2Streamer(
            client=self,
            command_ip=command_ip,
            command_port=command_port,
            stream_hz=stream_hz,
            action_gap_s=action_gap_s,
        )

    def create_trackc_streamer(
        self,
        command_ip="127.0.0.1",
        command_port=9200,
        stream_hz=500,
        samples_per_segment=50,
        path_mode="linear",
    ):
        """Create a UDP Cartesian-reference streamer for trackC.

        trackC streams row-major 4x4 TCP pose references over UDP while the
        server runs a Cartesian impedance controller.

        Args:
            command_ip: Server IP address that receives trackC UDP T_ref packets.
            command_port: Server UDP port for trackC commands.
            stream_hz: UDP command streaming frequency in Hz.
            samples_per_segment: Number of interpolated samples per action
                segment.
            path_mode: "linear" or "catmull_rom" Cartesian interpolation.

        Returns:
            TrackCStreamer instance. Use start(), update_waypoints(), finish(),
            stop(), and close() on it.
        """

        if not self.initialized:
            raise RuntimeError("Client is not initialized")

        return TrackCStreamer(
            client=self,
            command_ip=command_ip,
            command_port=command_port,
            stream_hz=stream_hz,
            samples_per_segment=samples_per_segment,
            path_mode=path_mode,
        )

    def create_trackc2_streamer(
        self,
        command_ip="127.0.0.1",
        command_port=9210,
        stream_hz=100,
        action_gap_s=0.01,
    ):
        """Create an async UDP Cartesian-target streamer for trackC2.

        trackC2 sends raw T_target samples over UDP. The client owns chunk
        timing, while the server smooths the latest target with a second-order
        dynamic filter before Cartesian impedance tracking.
        """

        if not self.initialized:
            raise RuntimeError("Client is not initialized")

        return TrackC2Streamer(
            client=self,
            command_ip=command_ip,
            command_port=command_port,
            stream_hz=stream_hz,
            action_gap_s=action_gap_s,
        )
    

    
    def movecart_no_queue(
        self,
        T_d,
        stiffness,
        vmax_linear,
        acc_max_linear,
        vmax_angular,
        acc_max_angular,
        nullspace_stiffness,
    ):
        """Execute a direct single-target Cartesian move RPC call.

        Args:
            T_d: 16-element row-major 4x4 target pose in base/world frame.
            stiffness: 6x6 Cartesian stiffness matrix.
            vmax_linear: Linear velocity limit in m/s.
            acc_max_linear: Linear acceleration limit in m/s^2.
            vmax_angular: Angular velocity limit in rad/s.
            acc_max_angular: Angular acceleration limit in rad/s^2.
            nullspace_stiffness: Joint nullspace stiffness.
        """

        return self.proxy.moveCartesian(
            T_d,
            stiffness,
            vmax_linear,
            acc_max_linear,
            vmax_angular,
            acc_max_angular,
            nullspace_stiffness,
        )


    def movecart_queue(
        self,
        T_d,
        stiffness,
        vmax_linear,
        acc_max_linear,
        vmax_angular,
        acc_max_angular,
        nullspace_stiffness,
    ):
        """Queue a single-target Cartesian move command.

        Args:
            T_d: 16-element row-major 4x4 target pose in base/world frame.
            stiffness: 6x6 Cartesian stiffness matrix.
            vmax_linear: Linear velocity limit in m/s.
            acc_max_linear: Linear acceleration limit in m/s^2.
            vmax_angular: Angular velocity limit in rad/s.
            acc_max_angular: Angular acceleration limit in rad/s^2.
            nullspace_stiffness: Joint nullspace stiffness.
        """

        return self.proxy.moveCartesian_queue(
            T_d,
            stiffness,
            vmax_linear,
            acc_max_linear,
            vmax_angular,
            acc_max_angular,
            nullspace_stiffness,
        )
    def movecart_path_no_queue(
        self,
        waypoints,
        samples_per_segment,
        path_mode,
        stiffness,
        vmax_linear,
        vmax_angular,
        nullspace_stiffness,
        acc_max_linear=None,
        acc_max_angular=None,
    ):
        """Execute a direct Cartesian path RPC call for pose waypoints.

        Args:
            waypoints: List of 16-element row-major 4x4 poses.
            samples_per_segment: Number of base replay samples per segment.
            path_mode: "linear" or "catmull_rom".
            stiffness: 6x6 Cartesian stiffness matrix.
            vmax_linear: Linear velocity limit checked during replay in m/s.
            vmax_angular: Angular velocity limit checked during replay in rad/s.
            nullspace_stiffness: Joint nullspace stiffness.
            acc_max_linear: Ignored by moveCartesianPath.
            acc_max_angular: Ignored by moveCartesianPath.
        """

        if acc_max_linear is not None or acc_max_angular is not None:
            print(
                "Warning: acc_max_linear/acc_max_angular are ignored by "
                "moveCartesianPath. Path replay only checks "
                "vmax_linear/vmax_angular."
            )

        if acc_max_linear is None:
            acc_max_linear = 0.0
        if acc_max_angular is None:
            acc_max_angular = 0.0

        return self.proxy.moveCartesianPath(
            waypoints,
            samples_per_segment,
            path_mode,
            stiffness,
            vmax_linear,
            acc_max_linear,
            vmax_angular,
            acc_max_angular,
            nullspace_stiffness,
        )


    def movecart_path_queue(
        self,
        waypoints,
        samples_per_segment,
        path_mode,
        stiffness,
        vmax_linear,
        vmax_angular,
        nullspace_stiffness,
        acc_max_linear=None,
        acc_max_angular=None,
    ):
        """Queue a Cartesian path command for pose waypoints.

        Args:
            waypoints: List of 16-element row-major 4x4 poses.
            samples_per_segment: Number of base replay samples per segment.
            path_mode: "linear" or "catmull_rom".
            stiffness: 6x6 Cartesian stiffness matrix.
            vmax_linear: Linear velocity limit checked during replay in m/s.
            vmax_angular: Angular velocity limit checked during replay in rad/s.
            nullspace_stiffness: Joint nullspace stiffness.
            acc_max_linear: Ignored by moveCartesianPath_queue.
            acc_max_angular: Ignored by moveCartesianPath_queue.
        """

        if acc_max_linear is not None or acc_max_angular is not None:
            print(
                "Warning: acc_max_linear/acc_max_angular are ignored by "
                "moveCartesianPath_queue. Path replay only checks "
                "vmax_linear/vmax_angular."
            )

        if acc_max_linear is None:
            acc_max_linear = 0.0
        if acc_max_angular is None:
            acc_max_angular = 0.0

        return self.proxy.moveCartesianPath_queue(
            waypoints,
            samples_per_segment,
            path_mode,
            stiffness,
            vmax_linear,
            acc_max_linear,
            vmax_angular,
            acc_max_angular,
            nullspace_stiffness,
        )

    def movecart(
        self,
        target,
        stiffness,
        vmax_linear,
        acc_max_linear=None,
        vmax_angular=0.5,
        acc_max_angular=None,
        nullspace_stiffness=1.0,
        queue=False,
        samples_per_segment=10,
        path_mode="linear",
    ):
        """Dispatch Cartesian motion to single-target or waypoint-path RPC.

        Args:
            target: Either a 16-element row-major pose or an Nx16 waypoint list.
            stiffness: 6x6 Cartesian stiffness matrix.
            vmax_linear: Linear velocity limit in m/s.
            acc_max_linear: Required for a single pose; ignored for paths.
            vmax_angular: Angular velocity limit in rad/s.
            acc_max_angular: Required for a single pose; ignored for paths.
            nullspace_stiffness: Joint nullspace stiffness.
            queue: If True, enqueue instead of executing immediately.
            samples_per_segment: Base replay samples per path segment.
            path_mode: "linear" or "catmull_rom" for waypoint paths.
        """

        if not isinstance(target, (list, tuple)):
            raise TypeError("target must be a list or tuple")
        if len(target) == 16 and all(isinstance(x, (int, float)) for x in target):
            
            if acc_max_linear is None or acc_max_angular is None:
                raise ValueError(
                    "acc_max_linear and acc_max_angular are required for single moveCartesian."
                )

            if queue:
                return self.movecart_queue(
                    target,
                    stiffness,
                    vmax_linear,
                    acc_max_linear,
                    vmax_angular,
                    acc_max_angular,
                    nullspace_stiffness,
                )

            return self.movecart_no_queue(
                target,
                stiffness,
                vmax_linear,
                acc_max_linear,
                vmax_angular,
                acc_max_angular,
                nullspace_stiffness,
            )


        if len(target) > 0 and all(
            isinstance(row, (list, tuple)) and len(row) == 16
            for row in target
        ):
            if queue:
                return self.movecart_path_queue(
                    waypoints=target,
                    samples_per_segment=samples_per_segment,
                    path_mode=path_mode,
                    stiffness=stiffness,
                    vmax_linear=vmax_linear,
                    vmax_angular=vmax_angular,
                    nullspace_stiffness=nullspace_stiffness,
                    acc_max_linear=acc_max_linear,
                    acc_max_angular=acc_max_angular,
                )

            return self.movecart_path_no_queue(
                waypoints=target,
                samples_per_segment=samples_per_segment,
                path_mode=path_mode,
                stiffness=stiffness,
                vmax_linear=vmax_linear,
                vmax_angular=vmax_angular,
                nullspace_stiffness=nullspace_stiffness,
                acc_max_linear=acc_max_linear,
                acc_max_angular=acc_max_angular,
            )


        raise ValueError("target must be either a 16-element pose or an Nx16 waypoint list")

    def stop_arm_motion(self):
        """Stop current arm motion and clear queued arm commands."""

        return self.proxy.stopArmMotion()

    def set_slowdown_factor(self, slowdown_factor):
        """Set runtime arm-motion slowdown through XML-RPC.

        Args:
            slowdown_factor: Replay time multiplier. 1.0 is the original speed;
                10.0 advances motion references about 10 times slower.
        """

        if not np.isfinite(slowdown_factor) or slowdown_factor < 1.0:
            raise ValueError("slowdown_factor must be greater than or equal to 1.0")

        return self.proxy.setSlowdownFactor(float(slowdown_factor))

    def get_slowdown_factor(self):
        """Return the current runtime arm-motion slowdown."""

        return self.proxy.getSlowdownFactor()

    def set_trackj2_filter_params(self, omega_n=10.0, zeta=1.0):
        """Set TrackJ2 second-order filter parameters."""

        if not np.isfinite(omega_n) or omega_n <= 0.0:
            raise ValueError("omega_n must be positive")
        if not np.isfinite(zeta) or zeta <= 0.0:
            raise ValueError("zeta must be positive")

        return self.proxy.setTrackJ2FilterParams(float(omega_n), float(zeta))

    def set_trackc2_filter_params(self, omega_n=10.0, zeta=1.0):
        """Set TrackC2 second-order filter parameters."""

        if not np.isfinite(omega_n) or omega_n <= 0.0:
            raise ValueError("omega_n must be positive")
        if not np.isfinite(zeta) or zeta <= 0.0:
            raise ValueError("zeta must be positive")

        return self.proxy.setTrackC2FilterParams(float(omega_n), float(zeta))

    def grasp_no_queue(self, width, speed, force, epsilon_inner, epsilon_outer):
        """Execute a direct gripper grasp RPC call.

        Args:
            width: Target grasp width in meters.
            speed: Gripper speed in m/s.
            force: Grasping force in newtons.
            epsilon_inner: Inner grasp tolerance in meters.
            epsilon_outer: Outer grasp tolerance in meters.
        """

        return self.proxy.graspO(width, speed, force, epsilon_inner, epsilon_outer)

    def grasp_queue(self, width, speed, force, epsilon_inner, epsilon_outer):
        """Queue a gripper grasp command.

        Args:
            width: Target grasp width in meters.
            speed: Gripper speed in m/s.
            force: Grasping force in newtons.
            epsilon_inner: Inner grasp tolerance in meters.
            epsilon_outer: Outer grasp tolerance in meters.
        """

        return self.proxy.graspO_queue(width, speed, force, epsilon_inner, epsilon_outer)
    
    def grasp(self, width, speed, force, epsilon_inner, epsilon_outer, queue=False):
        """Dispatch gripper grasp to direct or queued RPC.

        Args:
            width: Target grasp width in meters.
            speed: Gripper speed in m/s.
            force: Grasping force in newtons.
            epsilon_inner: Inner grasp tolerance in meters.
            epsilon_outer: Outer grasp tolerance in meters.
            queue: If True, enqueue instead of executing immediately.
        """

        if queue:
            return self.grasp_queue(width, speed, force, epsilon_inner, epsilon_outer)
        else:
            return self.grasp_no_queue(width, speed, force, epsilon_inner, epsilon_outer)
          
    
    
    
    
    def gripper_home_no_queue(self):
        """Execute a direct gripper homing RPC call."""

        return self.proxy.gripperHome()
    def gripper_home_queue(self):
        """Queue a gripper homing command."""

        return self.proxy.gripperHome_queue()
    def gripper_home(self, queue=False):
        """Dispatch gripper homing to direct or queued RPC.

        Args:
            queue: If True, enqueue instead of executing immediately.
        """

        if queue:
            return self.gripper_home_queue()
        else:
            return self.gripper_home_no_queue()
    
    
    def gripper_release_no_queue(self, speed):
        """Execute a direct gripper release/open RPC call.

        Args:
            speed: Opening speed in m/s.
        """

        return self.proxy.gripperRelease(speed)

    def gripper_release_queue(self, speed):
        """Queue a gripper release/open command.

        Args:
            speed: Opening speed in m/s.
        """

        return self.proxy.gripperRelease_queue(speed)

    def gripper_release(self, speed, queue=False):
        """Dispatch gripper release/open to direct or queued RPC.

        Args:
            speed: Opening speed in m/s.
            queue: If True, enqueue instead of executing immediately.
        """

        if queue:
            return self.gripper_release_queue(speed)
        else:
            return self.gripper_release_no_queue(speed)
    def _rpy_to_rotation_matrix(self, roll, pitch, yaw):
        """Build a rotation matrix from roll, pitch, and yaw angles in radians."""

        cx, sx = np.cos(roll), np.sin(roll)
        cy, sy = np.cos(pitch), np.sin(pitch)
        cz, sz = np.cos(yaw), np.sin(yaw)

        Rx = np.array([
            [1.0, 0.0, 0.0],
            [0.0, cx, -sx],
            [0.0, sx, cx],
        ])

        Ry = np.array([
            [cy, 0.0, sy],
            [0.0, 1.0, 0.0],
            [-sy, 0.0, cy],
        ])

        Rz = np.array([
            [cz, -sz, 0.0],
            [sz, cz, 0.0],
            [0.0, 0.0, 1.0],
        ])

        return Rz @ Ry @ Rx

    def _make_relative_transform(self, dx, dy, dz, droll, dpitch, dyaw):
        """Build a 4x4 transform from meters translation and radians RPY."""

        T_relative = np.eye(4)
        T_relative[:3, :3] = self._rpy_to_rotation_matrix(droll, dpitch, dyaw)
        T_relative[0, 3] = dx
        T_relative[1, 3] = dy
        T_relative[2, 3] = dz
        return T_relative

    def get_current_tcp_pose(
        self,
        robot_model,
        frame_name=None,
        timeout=1.0,
        flush=False,
    ):
        """Read UDP q and return the current TCP pose from FK.

        Args:
            robot_model: RobotModel used for forward kinematics.
            frame_name: Optional frame name. Defaults to default_frame_name.
            timeout: Maximum wait time for UDP state in seconds.
            flush: If True, clear old UDP packets before reading.

        Returns:
            4x4 TCP pose matrix in base/world frame.
        """

        if flush:
            self.empty_buffer()

        state = self.wait_for_first_udp(timeout=timeout)
        q = state["q"]

        return self.get_tcp_pose_from_q(
            robot_model,
            q,
            frame_name=frame_name,
        )

    
    
    def get_tcp_pose_from_q(self, robot_model, q, frame_name=None):
        """Return the FK pose for q using the default or given TCP frame.

        Args:
            robot_model: RobotModel used for forward kinematics.
            q: 7-element joint vector in radians.
            frame_name: Optional frame name. Defaults to default_frame_name.

        Returns:
            4x4 TCP pose matrix in base/world frame.
        """

        if frame_name is None:
            frame_name = self.default_frame_name

        pose = robot_model.get_frame_pose(q, frame_name=frame_name)
        return pose["T"]


    def get_all_basic_terms(self, robot_model, q, dq, frame_name=None):
        """Return kinematic and dynamic terms for q/dq at the TCP frame.

        Args:
            robot_model: RobotModel used for Pinocchio terms.
            q: 7-element joint vector in radians.
            dq: 7-element joint velocity vector in rad/s.
            frame_name: Optional frame name. Defaults to default_frame_name.
        """

        if frame_name is None:
            frame_name = self.default_frame_name

        return robot_model.get_all_basic_terms(q, dq, frame_name=frame_name)



    def movecart_relative(
        self,
        robot_model,
        dx=0.0,
        dy=0.0,
        dz=0.0,
        droll=0.0,
        dpitch=0.0,
        dyaw=0.0,
        stiffness=None,
        vmax=0.03,
        acc_max=0.05,
        nullspace_stiffness=10.0,
        frame_name=None,
        timeout=1.0,
        queue=False,
        reference_frame="tool",
    ):
        """Move TCP relative to its current pose in tool or base/world frame.

        Args:
            robot_model: RobotModel used to compute the current TCP pose.
            dx, dy, dz: Relative translation in meters.
            droll, dpitch, dyaw: Relative rotation in radians.
            stiffness: Optional 6x6 Cartesian stiffness matrix.
            vmax: Linear and angular velocity limit, in m/s and rad/s.
            acc_max: Linear and angular acceleration limit, in m/s^2 and rad/s^2.
            nullspace_stiffness: Joint nullspace stiffness.
            frame_name: Optional TCP frame name.
            timeout: Maximum wait time for UDP state in seconds.
            queue: If True, enqueue instead of executing immediately.
            reference_frame: "tool" applies the delta in TCP coordinates;
                "world" applies it in base/world coordinates.
        """

        if stiffness is None:
            stiffness = [
                [25.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 25.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 25.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 5.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 5.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 5.0],
            ]

        if reference_frame not in ("tool", "world"):
            raise ValueError("reference_frame must be 'tool' or 'world'")

        T_current = self.get_current_tcp_pose(
            robot_model,
            frame_name=frame_name,
            timeout=timeout,
        )

        T_relative = self._make_relative_transform(
            dx, dy, dz, droll, dpitch, dyaw
        )

        if reference_frame == "tool":
            T_target = T_current @ T_relative
        else:
            T_target = T_relative @ T_current

        T_target_flat = T_target.reshape(-1).tolist()

        return self.movecart(
            T_target_flat,
            stiffness=stiffness,
            vmax_linear=vmax,
            acc_max_linear=acc_max,
            vmax_angular=vmax,
            acc_max_angular=acc_max,
            nullspace_stiffness=nullspace_stiffness,
            queue=queue,
        )

    def movecart_relative_transform(
        self,
        robot_model,
        T_relative,
        stiffness=None,
        vmax=0.03,
        acc_max=0.05,
        nullspace_stiffness=10.0,
        frame_name=None,
        timeout=1.0,
        queue=False,
        reference_frame="tool",
    ):
        """Move TCP by a user-provided 4x4 relative transform.

        Args:
            robot_model: RobotModel used to compute the current TCP pose.
            T_relative: 4x4 relative transform matrix.
            stiffness: Optional 6x6 Cartesian stiffness matrix.
            vmax: Linear and angular velocity limit, in m/s and rad/s.
            acc_max: Linear and angular acceleration limit, in m/s^2 and rad/s^2.
            nullspace_stiffness: Joint nullspace stiffness.
            frame_name: Optional TCP frame name.
            timeout: Maximum wait time for UDP state in seconds.
            queue: If True, enqueue instead of executing immediately.
            reference_frame: "tool" applies T_relative after current TCP pose;
                "world" applies T_relative before current TCP pose.
        """

        if stiffness is None:
            stiffness = [
                [25.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 25.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 25.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 5.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 5.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 5.0],
            ]

        if reference_frame not in ("tool", "world"):
            raise ValueError("reference_frame must be 'tool' or 'world'")

        T_relative = np.asarray(T_relative, dtype=float)

        if T_relative.shape != (4, 4):
            raise ValueError(
                f"T_relative must have shape (4, 4), got {T_relative.shape}"
            )

        T_current = self.get_current_tcp_pose(
            robot_model,
            frame_name=frame_name,
            timeout=timeout,
        )

        if reference_frame == "tool":
            T_target = T_current @ T_relative
        else:
            T_target = T_relative @ T_current

        T_target_flat = T_target.reshape(-1).tolist()

        return self.movecart(
            T_target_flat,
            stiffness=stiffness,
            vmax_linear=vmax,
            acc_max_linear=acc_max,
            vmax_angular=vmax,
            acc_max_angular=acc_max,
            nullspace_stiffness=nullspace_stiffness,
            queue=queue,
        )

    def get_gripper_state(self):
        """Return the current gripper state as a readable string."""

        state = self.proxy.getGripperState()
        mapping = {
            0: "IDLE",
            1: "MOVING",
            2: "ERROR",
            3: "WIDTH_TOO_LARGE",
            4: "HOLDING",
            5: "OPEN_FAILED",
        }
        return mapping.get(state, f"UNKNOWN({state})")

    def get_gripper_width(self):
        """Return the cached gripper width in meters."""
        return float(self.proxy.getGripperWidth())


    def get_arm_state(self):
        """Return the current arm state as a readable string."""

        state = self.proxy.getArmState()
        mapping = {
            0: "IDLE",
            1: "MOVING",
            2: "ERROR",
        }
        return mapping.get(state, f"UNKNOWN({state})")

    def wait_until_arm_moving_finished(self, timeout=None, poll_interval=0.02):
        """Wait until the arm leaves MOVING and return its final state.

        Args:
            timeout: Maximum wait time in seconds, or None to wait forever.
            poll_interval: Time between RPC state checks in seconds.
        """

        start_time = time.monotonic()

        while True:
            arm_state = self.get_arm_state()

            if arm_state != "MOVING":
                return arm_state

            if timeout is not None and (time.monotonic() - start_time) >= timeout:
                raise TimeoutError(f"wait_until_arm_moving_finished timed out after {timeout} seconds")

            time.sleep(poll_interval)
            
    def wait_until_arm_idle_ok(self, timeout=None, poll_interval=0.02):
        """Wait until arm motion ends and raise if the arm enters ERROR.

        Args:
            timeout: Maximum wait time in seconds, or None to wait forever.
            poll_interval: Time between RPC state checks in seconds.
        """

        arm_state = self.wait_until_arm_moving_finished(
            timeout=timeout,
            poll_interval=poll_interval,
        )

        if arm_state == "ERROR":
            raise RuntimeError("Arm entered ERROR state")

        return arm_state

    def wait_until_gripper_moving_finished(self, timeout=None, poll_interval=0.02):
        """Wait until the gripper leaves MOVING and return its final state.

        Args:
            timeout: Maximum wait time in seconds, or None to wait forever.
            poll_interval: Time between RPC state checks in seconds.
        """

        start_time = time.monotonic()

        while True:
            gripper_state = self.get_gripper_state()

     
            if gripper_state != "MOVING":
                return gripper_state

            if timeout is not None and (time.monotonic() - start_time) >= timeout:
                raise TimeoutError(f"wait_until_gripper_moving_finished timed out after {timeout} seconds")

            time.sleep(poll_interval)
    def wait_until_motion_done(self, timeout=None, poll_interval=0.02):
        """Wait until both arm and gripper are no longer MOVING.

        Args:
            timeout: Maximum wait time in seconds, or None to wait forever.
            poll_interval: Time between RPC state checks in seconds.

        Returns:
            Dict with arm_state and gripper_state strings.
        """

        start_time = time.monotonic()

        while True:
            arm_state = self.get_arm_state()
            gripper_state = self.get_gripper_state()

            arm_done = arm_state != "MOVING"
            gripper_done = gripper_state != "MOVING"

            if arm_done and gripper_done:
                return {
                    "arm_state": arm_state,
                    "gripper_state": gripper_state,
                }

            if timeout is not None and (time.monotonic() - start_time) >= timeout:
                raise TimeoutError(f"wait_until_motion_done timed out after {timeout} seconds")

            time.sleep(poll_interval)

    
    def get_observation(self):
        """Return a compact latest-frame observation from one UDP frame.

        For VLA/policy inference with temporal context, prefer get_padded_data().
        For reliable command completion checks, use get_arm_state(),
        get_gripper_state(), or the wait_until_* helpers.

        Returns:
            Dict with q, dq, K_F_ext_hat, arm/gripper state, state ids,
            and UDP freshness;
            None when no usable UDP state is available.
        """

        state, info = self.get_latest_state(allow_stale=True)
        if state is None:
            return None

        return {
            "q": state["q"],
            "dq": state["dq"],
            "K_F_ext_hat": state["K_F_ext_hat"],
            "arm_state": state["arm_state"],
            "gripper_state": state["gripper_state"],
            "arm_state_id": state["arm_state_id"],
            "gripper_state_id": state["gripper_state_id"],
            "state_age": info["age"],
            "state_level": info["level"],
            "state_is_new": info["is_new"],
        }

    def get_observation_with_kinematics(self, robot_model, frame_name=None):
        """Return observation plus TCP pose, Jacobian, M, g, and c.

        Args:
            robot_model: RobotModel used for FK and dynamics terms.
            frame_name: Optional TCP frame name. Defaults to default_frame_name.
        """

        if frame_name is None:
            frame_name = self.default_frame_name

        state, info = self.get_latest_state(allow_stale=True)
        if state is None:
            return None

        q = state["q"]
        dq = state["dq"]

        terms = robot_model.get_all_basic_terms(q, dq, frame_name=frame_name)

        return {
            "q": state["q"],
            "dq": state["dq"],
            "K_F_ext_hat": state["K_F_ext_hat"],
            "arm_state": state["arm_state"],
            "gripper_state": state["gripper_state"],
            "arm_state_id": state["arm_state_id"],
            "gripper_state_id": state["gripper_state_id"],
            "state_age": info["age"],
            "state_level": info["level"],
            "state_is_new": info["is_new"],
            "ee_translation": terms["ee_translation"].tolist(),
            "ee_rotation": terms["ee_rotation"].tolist(),
            "ee_T": terms["ee_T"].tolist(),
            "J": terms["J"].tolist(),
            "M": terms["M"].tolist(),
            "g": terms["g"].tolist(),
            "c": terms["c"].tolist(),
        }
        
        


        
        
    def recover_system(self):
        """Call server-side recoverSystem and return its RPC result code."""

        return self.proxy.recoverSystem()

    def decode_rpc_result(self, result):
        """Decode a numeric RPC result code into a readable string."""

        mapping = {
            0: "OK",
            -2: "DENIED_MOVING",
            -3: "DENIED_ERROR",
            -4: "EXECUTION_FAILED",
        }
        return mapping.get(result, f"UNKNOWN({result})")
    
    def print_rpc_result(self, result, prefix="RPC"):
        """Print an RPC result code with its decoded meaning."""

        print(f"{prefix} result: {result} ({self.decode_rpc_result(result)})")
        
    def set_idle_state_poll_frequency(self, frequency_hz: int):
        """Set server idle-state polling frequency in Hz.

        Higher values can make idle UDP feedback fresher, but also increase
        server CPU/network work. During robot motion, the control callback is
        still the primary source of streamed UDP state.
        """

        return self.proxy.setIdleStatePollFrequency(frequency_hz)
