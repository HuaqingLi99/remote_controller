import itertools
import socket
import struct
import threading
import time

import numpy as np

TRACKJ_PACKET_FORMAT = "<Q 7d"
TRACKC_PACKET_FORMAT = "<Q 16d"


def interpolate_joint_vec(a, b, alpha):
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    return a + alpha * (b - a)


def joint_distance(a, b):
    return float(np.linalg.norm(np.asarray(b, dtype=float) - np.asarray(a, dtype=float)))


def connect_linear_joint_path(waypoints, samples_per_segment):
    if len(waypoints) < 2:
        raise ValueError("At least 2 waypoints are required")
    if samples_per_segment <= 0:
        raise ValueError("samples_per_segment must be positive")

    result = []
    for i in range(len(waypoints) - 1):
        start = np.asarray(waypoints[i], dtype=float)
        end = np.asarray(waypoints[i + 1], dtype=float)

        for j in range(samples_per_segment):
            u = j / samples_per_segment
            result.append(interpolate_joint_vec(start, end, u))

    result.append(np.asarray(waypoints[-1], dtype=float))
    return result


def resample_by_joint_distance(dense_path, output_samples):
    dense_path = [np.asarray(p, dtype=float) for p in dense_path]
    if len(dense_path) < 2:
        raise ValueError("dense_path must have at least 2 points")
    if output_samples < 2:
        raise ValueError("output_samples must be >= 2")

    total_length = sum(
        joint_distance(dense_path[i - 1], dense_path[i])
        for i in range(1, len(dense_path))
    )
    if total_length <= 1e-12:
        raise ValueError("path length is too small")

    target_ds = total_length / (output_samples - 1)
    result = [dense_path[0]]
    accumulated_s = 0.0
    next_target_s = target_ds

    for i in range(1, len(dense_path)):
        seg_start = dense_path[i - 1]
        seg_end = dense_path[i]
        segment_len = joint_distance(seg_start, seg_end)
        if segment_len <= 1e-12:
            continue

        while (
            accumulated_s + segment_len >= next_target_s
            and len(result) + 1 < output_samples
        ):
            alpha = (next_target_s - accumulated_s) / segment_len
            result.append(interpolate_joint_vec(seg_start, seg_end, alpha))
            next_target_s += target_ds

        accumulated_s += segment_len

    result.append(dense_path[-1])
    return result


def build_linear_joint_trajectory_with_action_index(
    q_start,
    waypoints,
    samples_per_segment,
):
    if samples_per_segment <= 0:
        raise ValueError("samples_per_segment must be positive")

    if len(waypoints) == 0:
        return [np.asarray(q_start, dtype=float)], []

    trajectory = []
    sample_action_indices = []

    path = [np.asarray(q_start, dtype=float)]
    path.extend(np.asarray(wp, dtype=float) for wp in waypoints)

    for action_idx in range(len(waypoints)):
        start = path[action_idx]
        end = path[action_idx + 1]

        for j in range(samples_per_segment):
            alpha = j / samples_per_segment
            trajectory.append(interpolate_joint_vec(start, end, alpha))
            sample_action_indices.append(action_idx)

    trajectory.append(path[-1])
    sample_action_indices[-1] = len(waypoints) - 1

    return trajectory, sample_action_indices


class TrajectoryManager:
    def __init__(self, samples_per_segment=50):
        self.lock = threading.Lock()
        self.samples_per_segment = samples_per_segment

        self.active_waypoints = []
        self.active_trajectory = []
        self.sample_action_indices = []

        self.index = 0
        self.last_sent_q = None
        self.completed = True

    def _build_trajectory(self, q_start, waypoints):
        return build_linear_joint_trajectory_with_action_index(
            q_start,
            waypoints,
            self.samples_per_segment,
        )

    def _join_waypoints(self, first, second):
        first = [np.asarray(wp, dtype=float) for wp in first]
        second = [np.asarray(wp, dtype=float) for wp in second]

        if first and second and joint_distance(first[-1], second[0]) <= 1e-12:
            return first + second[1:]

        return first + second

    def _remaining_waypoints_locked(self):
        if self.completed or not self.active_waypoints:
            return []

        if not self.sample_action_indices:
            return list(self.active_waypoints)

        next_idx = min(self.index, len(self.sample_action_indices) - 1)
        action_idx = self.sample_action_indices[next_idx]

        return list(self.active_waypoints[action_idx:])

    def _set_new_plan_locked(self, waypoints):
        waypoints = [np.asarray(wp, dtype=float) for wp in waypoints]

        if len(waypoints) == 0:
            q_hold = np.asarray(self.last_sent_q, dtype=float)
            self.active_waypoints = []
            self.active_trajectory = [q_hold]
            self.sample_action_indices = []
            self.index = 0
            self.completed = True
            return

        trajectory, sample_action_indices = self._build_trajectory(
            self.last_sent_q,
            waypoints,
        )

        self.active_waypoints = waypoints
        self.active_trajectory = trajectory
        self.sample_action_indices = sample_action_indices
        self.index = 0
        self.completed = False

    def update_plan(self, waypoints, merge_mode="merge_remaining", merge_fn=None):
        with self.lock:
            if self.last_sent_q is None:
                raise RuntimeError("last_sent_q is not initialized")

            new_waypoints = [np.asarray(wp, dtype=float) for wp in waypoints]
            remaining_waypoints = self._remaining_waypoints_locked()

            if merge_fn is not None:
                merged_waypoints = merge_fn(
                    last_sent_q=np.asarray(self.last_sent_q, dtype=float),
                    remaining_waypoints=list(remaining_waypoints),
                    new_waypoints=list(new_waypoints),
                    active_waypoints=list(self.active_waypoints),
                    current_sample_index=self.index,
                )
            elif merge_mode == "replace":
                merged_waypoints = new_waypoints
            elif merge_mode == "merge_remaining":
                merged_waypoints = self._join_waypoints(
                    remaining_waypoints,
                    new_waypoints,
                )
            else:
                raise ValueError("merge_mode must be 'replace' or 'merge_remaining'")

            self._set_new_plan_locked(merged_waypoints)

    def replace(self, waypoints):
        self.update_plan(waypoints, merge_mode="replace")

    def merge_remaining(self, waypoints):
        self.update_plan(waypoints, merge_mode="merge_remaining")

    def initialize_hold(self, q_current):
        with self.lock:
            q_current = np.asarray(q_current, dtype=float)
            self.active_waypoints = []
            self.active_trajectory = [q_current]
            self.sample_action_indices = []
            self.index = 0
            self.last_sent_q = q_current
            self.completed = True

    def next_sample(self):
        with self.lock:
            if not self.active_trajectory:
                return None, None

            idx = min(self.index, len(self.active_trajectory) - 1)
            q_ref = np.asarray(self.active_trajectory[idx], dtype=float)

            if self.index < len(self.active_trajectory) - 1:
                self.index += 1
                self.completed = False
            else:
                self.completed = True

            self.last_sent_q = q_ref
            return idx, q_ref


class TrackJStreamer:
    def __init__(
        self,
        client,
        command_ip,
        command_port,
        stream_hz=500,
        samples_per_segment=50,
    ):
        if stream_hz <= 0:
            raise ValueError("stream_hz must be positive")

        self.client = client
        self.command_addr = (command_ip, int(command_port))
        self.stream_hz = float(stream_hz)
        self.manager = TrajectoryManager(
            samples_per_segment=samples_per_segment,
        )

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq_counter = itertools.count(1)

        self.running = threading.Event()
        self.thread = None
        self.started = False
        self.closed = False

    def _send_packet(self, q_ref):
        packet = struct.pack(
            TRACKJ_PACKET_FORMAT,
            int(next(self.seq_counter)),
            *[float(x) for x in q_ref],
        )
        self.sock.sendto(packet, self.command_addr)

    def start(self, q_start, stiffness):
        if self.closed:
            raise RuntimeError("TrackJStreamer is closed")
        if self.started:
            raise RuntimeError("TrackJStreamer is already started")

        self.manager.initialize_hold(q_start)

        result = self.client.proxy.startTrackJ(
            int(self.command_addr[1]),
            float(self.stream_hz),
            stiffness,
        )
        if result != 0:
            raise RuntimeError(f"startTrackJ failed: {result}")

        self.started = True
        self.running.set()
        self.thread = threading.Thread(
            target=self._send_loop,
            name="TrackJUDPSender",
            daemon=True,
        )
        self.thread.start()

    def update_waypoints(self, waypoints, merge_mode="merge_remaining", merge_fn=None):
        self.manager.update_plan(
            waypoints,
            merge_mode=merge_mode,
            merge_fn=merge_fn,
        )

    def _send_loop(self):
        period = 1.0 / self.stream_hz
        next_time = time.perf_counter()

        while self.running.is_set():
            _, q_ref = self.manager.next_sample()
            if q_ref is not None:
                self._send_packet(q_ref)

            next_time += period
            sleep_time = next_time - time.perf_counter()

            if sleep_time > 0.0:
                time.sleep(sleep_time)
            else:
                next_time = time.perf_counter()

    def _request_server_stop(self):
        if self.started:
            self.client.stop_arm_motion()
            self.started = False

    def finish(self):
        self.stop()

    def stop(self):
        stop_error = None
        try:
            self._request_server_stop()
        except Exception as exc:
            stop_error = exc
        finally:
            self.running.clear()
            if self.thread is not None:
                self.thread.join(timeout=1.0)
                self.thread = None

        if stop_error is not None:
            raise stop_error

    def close(self):
        if not self.closed:
            try:
                self.stop()
            finally:
                self.sock.close()
                self.closed = True


class TrackJ2Streamer:
    def __init__(
        self,
        client,
        command_ip,
        command_port,
        stream_hz=100,
        action_gap_s=0.01,
    ):
        if stream_hz <= 0:
            raise ValueError("stream_hz must be positive")
        if action_gap_s <= 0:
            raise ValueError("action_gap_s must be positive")

        self.client = client
        self.command_addr = (command_ip, int(command_port))
        self.stream_hz = float(stream_hz)
        self.action_gap_s = float(action_gap_s)

        self.lock = threading.Lock()
        self.send_lock = threading.Lock()
        self.active_chunk = []
        self.index = 0
        self.last_send_time = None
        self.last_sent_q = None

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq_counter = itertools.count(1)

        self.running = threading.Event()
        self.thread = None
        self.started = False
        self.closed = False

    def _send_packet(self, q_target):
        q_target = np.asarray(q_target, dtype=float)
        if q_target.shape != (7,):
            raise ValueError("q_target must be a 7-element joint vector")

        with self.send_lock:
            packet = struct.pack(
                TRACKJ_PACKET_FORMAT,
                int(next(self.seq_counter)),
                *[float(x) for x in q_target],
            )
            self.sock.sendto(packet, self.command_addr)
            self.last_sent_q = q_target
            self.last_send_time = time.perf_counter()

    def start(self, q_start, stiffness, omega_n=10.0, zeta=1.0):
        if self.closed:
            raise RuntimeError("TrackJ2Streamer is closed")
        if self.started:
            raise RuntimeError("TrackJ2Streamer is already started")

        self.set_filter_params(omega_n, zeta)

        result = self.client.proxy.startTrackJ2(
            int(self.command_addr[1]),
            float(self.stream_hz),
            stiffness,
        )
        if result != 0:
            raise RuntimeError(f"startTrackJ2 failed: {result}")

        with self.lock:
            self.last_sent_q = np.asarray(q_start, dtype=float)
            self.last_send_time = None

        self.started = True
        self.running.set()
        self.thread = threading.Thread(
            target=self._send_loop,
            name="TrackJ2UDPSender",
            daemon=True,
        )
        self.thread.start()

    def set_filter_params(self, omega_n, zeta):
        result = self.client.proxy.setTrackJ2FilterParams(
            float(omega_n),
            float(zeta),
        )
        if result != 0:
            raise RuntimeError(f"setTrackJ2FilterParams failed: {result}")

    def set_action_gap(self, gap_s):
        if gap_s <= 0:
            raise ValueError("gap_s must be positive")
        with self.lock:
            self.action_gap_s = float(gap_s)

    def replace_chunk(self, chunk, send_first=True):
        chunk = [np.asarray(q, dtype=float) for q in chunk]
        for q in chunk:
            if q.shape != (7,):
                raise ValueError("Each TrackJ2 chunk target must have shape (7,)")

        first_target = None
        with self.lock:
            self.active_chunk = chunk
            self.index = 0
            if send_first and self.active_chunk:
                first_target = self.active_chunk[self.index]
                self.index += 1

        if first_target is not None:
            self._send_packet(first_target)

    def clear_chunk(self):
        with self.lock:
            self.active_chunk = []
            self.index = 0

    def send_target(self, q_target):
        self._send_packet(q_target)

    def send_next(self, force=False):
        target = None
        now = time.perf_counter()

        with self.lock:
            if self.index >= len(self.active_chunk):
                return False

            if not force and self.last_send_time is not None:
                if now - self.last_send_time < self.action_gap_s:
                    return False

            target = self.active_chunk[self.index]
            self.index += 1

        self._send_packet(target)
        return True

    def _send_loop(self):
        while self.running.is_set():
            self.send_next()
            time.sleep(0.001)

    def _request_server_stop(self):
        if self.started:
            self.client.stop_arm_motion()
            self.started = False

    def finish(self):
        self.stop()

    def stop(self):
        stop_error = None
        try:
            self._request_server_stop()
        except Exception as exc:
            stop_error = exc
        finally:
            self.running.clear()
            if self.thread is not None:
                self.thread.join(timeout=1.0)
                self.thread = None

        if stop_error is not None:
            raise stop_error

    def close(self):
        if not self.closed:
            try:
                self.stop()
            finally:
                self.sock.close()
                self.closed = True


def _as_pose_matrix(T):
    arr = np.asarray(T, dtype=float)
    if arr.shape == (4, 4):
        return arr.copy()
    if arr.size == 16:
        return arr.reshape((4, 4)).copy()
    raise ValueError("Cartesian pose must be a 4x4 matrix or 16-element row-major array")


def _normalize_quat(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if norm <= 1e-12:
        raise ValueError("Quaternion norm is too small")
    return q / norm


def _quat_from_matrix(R):
    R = np.asarray(R, dtype=float)
    trace = float(np.trace(R))

    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        return _normalize_quat([
            0.25 * s,
            (R[2, 1] - R[1, 2]) / s,
            (R[0, 2] - R[2, 0]) / s,
            (R[1, 0] - R[0, 1]) / s,
        ])

    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        return _normalize_quat([
            (R[2, 1] - R[1, 2]) / s,
            0.25 * s,
            (R[0, 1] + R[1, 0]) / s,
            (R[0, 2] + R[2, 0]) / s,
        ])

    if R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        return _normalize_quat([
            (R[0, 2] - R[2, 0]) / s,
            (R[0, 1] + R[1, 0]) / s,
            0.25 * s,
            (R[1, 2] + R[2, 1]) / s,
        ])

    s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
    return _normalize_quat([
        (R[1, 0] - R[0, 1]) / s,
        (R[0, 2] + R[2, 0]) / s,
        (R[1, 2] + R[2, 1]) / s,
        0.25 * s,
    ])


def _matrix_from_quat(q):
    w, x, y, z = _normalize_quat(q)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _slerp(q0, q1, alpha):
    q0 = _normalize_quat(q0)
    q1 = _normalize_quat(q1)

    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot

    if dot > 0.9995:
        return _normalize_quat(q0 + alpha * (q1 - q0))

    theta_0 = np.arccos(np.clip(dot, -1.0, 1.0))
    sin_theta_0 = np.sin(theta_0)
    theta = theta_0 * alpha
    sin_theta = np.sin(theta)

    s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return _normalize_quat((s0 * q0) + (s1 * q1))


def _quat_rotation_vector(q_start, q_goal):
    q_start = _normalize_quat(q_start)
    q_goal = _normalize_quat(q_goal)
    if np.dot(q_start, q_goal) < 0.0:
        q_goal = -q_goal

    w0, x0, y0, z0 = q_start
    w1, x1, y1, z1 = q_goal
    q_rel = _normalize_quat(
        [
            w0 * w1 + x0 * x1 + y0 * y1 + z0 * z1,
            w0 * x1 - x0 * w1 - y0 * z1 + z0 * y1,
            w0 * y1 + x0 * z1 - y0 * w1 - z0 * x1,
            w0 * z1 - x0 * y1 + y0 * x1 - z0 * w1,
        ]
    )
    if q_rel[0] < 0.0:
        q_rel = -q_rel

    angle = 2.0 * np.arctan2(np.linalg.norm(q_rel[1:]), q_rel[0])
    if abs(angle) < 1e-12:
        return np.zeros(3)

    axis = q_rel[1:] / np.linalg.norm(q_rel[1:])
    return axis * angle


def _pose_to_components(T):
    T = _as_pose_matrix(T)
    return T[:3, 3].copy(), _quat_from_matrix(T[:3, :3])


def _components_to_pose(p, q):
    T = np.eye(4, dtype=float)
    T[:3, :3] = _matrix_from_quat(q)
    T[:3, 3] = np.asarray(p, dtype=float)
    return T


def interpolate_cartesian_pose(a, b, alpha):
    p_a, q_a = _pose_to_components(a)
    p_b, q_b = _pose_to_components(b)
    if np.dot(q_a, q_b) < 0.0:
        q_b = -q_b

    p = p_a + alpha * (p_b - p_a)
    q = _slerp(q_a, q_b, alpha)
    return _components_to_pose(p, q)


def cartesian_pose_distance(a, b, rotation_weight=0.2):
    p_a, q_a = _pose_to_components(a)
    p_b, q_b = _pose_to_components(b)
    return float(np.linalg.norm(p_b - p_a) + rotation_weight * np.linalg.norm(
        _quat_rotation_vector(q_a, q_b)
    ))


def _tj_position(ti, pi, pj, alpha):
    return ti + max(float(np.linalg.norm(pj - pi)), 1e-12) ** alpha


def _catmull_rom_position(p0, p1, p2, p3, u, alpha=0.5):
    t0 = 0.0
    t1 = _tj_position(t0, p0, p1, alpha)
    t2 = _tj_position(t1, p1, p2, alpha)
    t3 = _tj_position(t2, p2, p3, alpha)
    t = t1 + u * (t2 - t1)

    A1 = ((t1 - t) / (t1 - t0)) * p0 + ((t - t0) / (t1 - t0)) * p1
    A2 = ((t2 - t) / (t2 - t1)) * p1 + ((t - t1) / (t2 - t1)) * p2
    A3 = ((t3 - t) / (t3 - t2)) * p2 + ((t - t2) / (t3 - t2)) * p3
    B1 = ((t2 - t) / (t2 - t0)) * A1 + ((t - t0) / (t2 - t0)) * A2
    B2 = ((t3 - t) / (t3 - t1)) * A2 + ((t - t1) / (t3 - t1)) * A3
    return ((t2 - t) / (t2 - t1)) * B1 + ((t - t1) / (t2 - t1)) * B2


def connect_linear_cartesian_path(waypoints, samples_per_segment):
    if len(waypoints) < 2:
        raise ValueError("At least 2 Cartesian waypoints are required")
    if samples_per_segment <= 0:
        raise ValueError("samples_per_segment must be positive")

    result = []
    for i in range(len(waypoints) - 1):
        for j in range(samples_per_segment):
            result.append(interpolate_cartesian_pose(
                waypoints[i],
                waypoints[i + 1],
                j / samples_per_segment,
            ))

    result.append(_as_pose_matrix(waypoints[-1]))
    return result


def connect_catmull_rom_cartesian_path(waypoints, samples_per_segment):
    if len(waypoints) < 2:
        raise ValueError("At least 2 Cartesian waypoints are required")
    if samples_per_segment <= 0:
        raise ValueError("samples_per_segment must be positive")
    if len(waypoints) <= 3:
        return connect_linear_cartesian_path(waypoints, samples_per_segment)

    poses = [_as_pose_matrix(wp) for wp in waypoints]
    padded = [poses[0], *poses, poses[-1]]

    dense_multiplier = 10
    dense_samples_per_segment = samples_per_segment * dense_multiplier
    dense_path = []

    for seg in range(len(padded) - 3):
        p0, q0 = _pose_to_components(padded[seg])
        p1, q1 = _pose_to_components(padded[seg + 1])
        p2, q2 = _pose_to_components(padded[seg + 2])
        p3, _ = _pose_to_components(padded[seg + 3])

        if np.dot(q1, q2) < 0.0:
            q2 = -q2

        for j in range(dense_samples_per_segment):
            u = j / dense_samples_per_segment
            p = _catmull_rom_position(p0, p1, p2, p3, u)
            q = _slerp(q1, q2, u)
            dense_path.append(_components_to_pose(p, q))

    dense_path.append(poses[-1])
    return dense_path


def resample_by_cartesian_distance(dense_path, output_samples, rotation_weight=0.2):
    dense_path = [_as_pose_matrix(p) for p in dense_path]
    if len(dense_path) < 2:
        raise ValueError("dense_path must have at least 2 points")
    if output_samples < 2:
        raise ValueError("output_samples must be >= 2")

    total_length = sum(
        cartesian_pose_distance(dense_path[i - 1], dense_path[i], rotation_weight)
        for i in range(1, len(dense_path))
    )
    if total_length <= 1e-12:
        raise ValueError("Cartesian path length is too small")

    target_ds = total_length / (output_samples - 1)
    result = [dense_path[0]]
    accumulated_s = 0.0
    next_target_s = target_ds

    for i in range(1, len(dense_path)):
        seg_start = dense_path[i - 1]
        seg_end = dense_path[i]
        segment_len = cartesian_pose_distance(seg_start, seg_end, rotation_weight)
        if segment_len <= 1e-12:
            continue

        while (
            accumulated_s + segment_len >= next_target_s
            and len(result) + 1 < output_samples
        ):
            alpha = (next_target_s - accumulated_s) / segment_len
            result.append(interpolate_cartesian_pose(seg_start, seg_end, alpha))
            next_target_s += target_ds

        accumulated_s += segment_len

    result.append(dense_path[-1])
    return result


def build_cartesian_trajectory_with_action_index(
    T_start,
    waypoints,
    samples_per_segment,
    path_mode="linear",
):
    if samples_per_segment <= 0:
        raise ValueError("samples_per_segment must be positive")

    if len(waypoints) == 0:
        return [_as_pose_matrix(T_start)], []

    raw_path = [_as_pose_matrix(T_start)]
    raw_path.extend(_as_pose_matrix(wp) for wp in waypoints)

    if path_mode == "linear":
        connected_path = connect_linear_cartesian_path(raw_path, samples_per_segment)
    elif path_mode == "catmull_rom":
        connected_path = connect_catmull_rom_cartesian_path(raw_path, samples_per_segment)
    else:
        raise ValueError("path_mode must be 'linear' or 'catmull_rom'")

    output_samples = (len(raw_path) - 1) * samples_per_segment + 1
    trajectory = resample_by_cartesian_distance(connected_path, output_samples)

    sample_action_indices = []
    for action_idx in range(len(waypoints)):
        sample_action_indices.extend([action_idx] * samples_per_segment)
    sample_action_indices.append(len(waypoints) - 1)

    return trajectory, sample_action_indices


class CartesianTrajectoryManager:
    def __init__(self, samples_per_segment=50, path_mode="linear"):
        self.lock = threading.Lock()
        self.samples_per_segment = samples_per_segment
        self.path_mode = path_mode

        self.active_waypoints = []
        self.active_trajectory = []
        self.sample_action_indices = []

        self.index = 0
        self.last_sent_T = None
        self.completed = True

    def _build_trajectory(self, T_start, waypoints):
        return build_cartesian_trajectory_with_action_index(
            T_start,
            waypoints,
            self.samples_per_segment,
            self.path_mode,
        )

    def _join_waypoints(self, first, second):
        first = [_as_pose_matrix(wp) for wp in first]
        second = [_as_pose_matrix(wp) for wp in second]

        if first and second and cartesian_pose_distance(first[-1], second[0]) <= 1e-12:
            return first + second[1:]

        return first + second

    def _remaining_waypoints_locked(self):
        if self.completed or not self.active_waypoints:
            return []

        if not self.sample_action_indices:
            return list(self.active_waypoints)

        next_idx = min(self.index, len(self.sample_action_indices) - 1)
        action_idx = self.sample_action_indices[next_idx]
        return list(self.active_waypoints[action_idx:])

    def _set_new_plan_locked(self, waypoints):
        waypoints = [_as_pose_matrix(wp) for wp in waypoints]

        if len(waypoints) == 0:
            T_hold = _as_pose_matrix(self.last_sent_T)
            self.active_waypoints = []
            self.active_trajectory = [T_hold]
            self.sample_action_indices = []
            self.index = 0
            self.completed = True
            return

        trajectory, sample_action_indices = self._build_trajectory(
            self.last_sent_T,
            waypoints,
        )

        self.active_waypoints = waypoints
        self.active_trajectory = trajectory
        self.sample_action_indices = sample_action_indices
        self.index = 0
        self.completed = False

    def update_plan(self, waypoints, merge_mode="merge_remaining", merge_fn=None):
        with self.lock:
            if self.last_sent_T is None:
                raise RuntimeError("last_sent_T is not initialized")

            new_waypoints = [_as_pose_matrix(wp) for wp in waypoints]
            remaining_waypoints = self._remaining_waypoints_locked()

            if merge_fn is not None:
                merged_waypoints = merge_fn(
                    last_sent_T=_as_pose_matrix(self.last_sent_T),
                    remaining_waypoints=list(remaining_waypoints),
                    new_waypoints=list(new_waypoints),
                    active_waypoints=list(self.active_waypoints),
                    current_sample_index=self.index,
                )
            elif merge_mode == "replace":
                merged_waypoints = new_waypoints
            elif merge_mode == "merge_remaining":
                merged_waypoints = self._join_waypoints(
                    remaining_waypoints,
                    new_waypoints,
                )
            else:
                raise ValueError("merge_mode must be 'replace' or 'merge_remaining'")

            self._set_new_plan_locked(merged_waypoints)

    def replace(self, waypoints):
        self.update_plan(waypoints, merge_mode="replace")

    def merge_remaining(self, waypoints):
        self.update_plan(waypoints, merge_mode="merge_remaining")

    def initialize_hold(self, T_current):
        with self.lock:
            T_current = _as_pose_matrix(T_current)
            self.active_waypoints = []
            self.active_trajectory = [T_current]
            self.sample_action_indices = []
            self.index = 0
            self.last_sent_T = T_current
            self.completed = True

    def next_sample(self):
        with self.lock:
            if not self.active_trajectory:
                return None, None

            idx = min(self.index, len(self.active_trajectory) - 1)
            T_ref = _as_pose_matrix(self.active_trajectory[idx])

            if self.index < len(self.active_trajectory) - 1:
                self.index += 1
                self.completed = False
            else:
                self.completed = True

            self.last_sent_T = T_ref
            return idx, T_ref


class TrackCStreamer:
    def __init__(
        self,
        client,
        command_ip,
        command_port,
        stream_hz=500,
        samples_per_segment=50,
        path_mode="linear",
    ):
        if stream_hz <= 0:
            raise ValueError("stream_hz must be positive")

        self.client = client
        self.command_addr = (command_ip, int(command_port))
        self.stream_hz = float(stream_hz)
        self.manager = CartesianTrajectoryManager(
            samples_per_segment=samples_per_segment,
            path_mode=path_mode,
        )

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq_counter = itertools.count(1)

        self.running = threading.Event()
        self.thread = None
        self.started = False
        self.closed = False

    def _send_packet(self, T_ref):
        T_ref = _as_pose_matrix(T_ref).reshape(-1)
        packet = struct.pack(
            TRACKC_PACKET_FORMAT,
            int(next(self.seq_counter)),
            *[float(x) for x in T_ref],
        )
        self.sock.sendto(packet, self.command_addr)

    def start(self, T_start, stiffness, nullspace_stiffness=1.0):
        if self.closed:
            raise RuntimeError("TrackCStreamer is closed")
        if self.started:
            raise RuntimeError("TrackCStreamer is already started")

        self.manager.initialize_hold(T_start)

        result = self.client.proxy.startTrackC(
            int(self.command_addr[1]),
            float(self.stream_hz),
            stiffness,
            float(nullspace_stiffness),
        )
        if result != 0:
            raise RuntimeError(f"startTrackC failed: {result}")

        self.started = True
        self.running.set()
        self.thread = threading.Thread(
            target=self._send_loop,
            name="TrackCUDPSender",
            daemon=True,
        )
        self.thread.start()

    def update_waypoints(self, waypoints, merge_mode="merge_remaining", merge_fn=None):
        self.manager.update_plan(
            waypoints,
            merge_mode=merge_mode,
            merge_fn=merge_fn,
        )

    def _send_loop(self):
        period = 1.0 / self.stream_hz
        next_time = time.perf_counter()

        while self.running.is_set():
            _, T_ref = self.manager.next_sample()
            if T_ref is not None:
                self._send_packet(T_ref)

            next_time += period
            sleep_time = next_time - time.perf_counter()

            if sleep_time > 0.0:
                time.sleep(sleep_time)
            else:
                next_time = time.perf_counter()

    def _request_server_stop(self):
        if self.started:
            self.client.stop_arm_motion()
            self.started = False

    def finish(self):
        self.stop()

    def stop(self):
        stop_error = None
        try:
            self._request_server_stop()
        except Exception as exc:
            stop_error = exc
        finally:
            self.running.clear()
            if self.thread is not None:
                self.thread.join(timeout=1.0)
                self.thread = None

        if stop_error is not None:
            raise stop_error

    def close(self):
        if not self.closed:
            try:
                self.stop()
            finally:
                self.sock.close()
                self.closed = True


class TrackC2Streamer:
    def __init__(
        self,
        client,
        command_ip,
        command_port,
        stream_hz=100,
        action_gap_s=0.01,
    ):
        if stream_hz <= 0:
            raise ValueError("stream_hz must be positive")
        if action_gap_s <= 0:
            raise ValueError("action_gap_s must be positive")

        self.client = client
        self.command_addr = (command_ip, int(command_port))
        self.stream_hz = float(stream_hz)
        self.action_gap_s = float(action_gap_s)

        self.lock = threading.Lock()
        self.send_lock = threading.Lock()
        self.active_chunk = []
        self.index = 0
        self.last_send_time = None
        self.last_sent_T = None

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq_counter = itertools.count(1)

        self.running = threading.Event()
        self.thread = None
        self.started = False
        self.closed = False

    def _send_packet(self, T_target):
        T_target = _as_pose_matrix(T_target)
        with self.send_lock:
            packet = struct.pack(
                TRACKC_PACKET_FORMAT,
                int(next(self.seq_counter)),
                *[float(x) for x in T_target.reshape(-1)],
            )
            self.sock.sendto(packet, self.command_addr)
            self.last_sent_T = T_target
            self.last_send_time = time.perf_counter()

    def start(
        self,
        T_start,
        stiffness,
        nullspace_stiffness=1.0,
        omega_n=10.0,
        zeta=1.0,
    ):
        if self.closed:
            raise RuntimeError("TrackC2Streamer is closed")
        if self.started:
            raise RuntimeError("TrackC2Streamer is already started")

        self.set_filter_params(omega_n, zeta)

        result = self.client.proxy.startTrackC2(
            int(self.command_addr[1]),
            float(self.stream_hz),
            stiffness,
            float(nullspace_stiffness),
        )
        if result != 0:
            raise RuntimeError(f"startTrackC2 failed: {result}")

        with self.lock:
            self.last_sent_T = _as_pose_matrix(T_start)
            self.last_send_time = None

        self.started = True
        self.running.set()
        self.thread = threading.Thread(
            target=self._send_loop,
            name="TrackC2UDPSender",
            daemon=True,
        )
        self.thread.start()

    def set_filter_params(self, omega_n, zeta):
        result = self.client.proxy.setTrackC2FilterParams(
            float(omega_n),
            float(zeta),
        )
        if result != 0:
            raise RuntimeError(f"setTrackC2FilterParams failed: {result}")

    def set_action_gap(self, gap_s):
        if gap_s <= 0:
            raise ValueError("gap_s must be positive")
        with self.lock:
            self.action_gap_s = float(gap_s)

    def replace_chunk(self, chunk, send_first=True):
        chunk = [_as_pose_matrix(T) for T in chunk]

        first_target = None
        with self.lock:
            self.active_chunk = chunk
            self.index = 0
            if send_first and self.active_chunk:
                first_target = self.active_chunk[self.index]
                self.index += 1

        if first_target is not None:
            self._send_packet(first_target)

    def clear_chunk(self):
        with self.lock:
            self.active_chunk = []
            self.index = 0

    def send_target(self, T_target):
        self._send_packet(T_target)

    def send_next(self, force=False):
        target = None
        now = time.perf_counter()

        with self.lock:
            if self.index >= len(self.active_chunk):
                return False

            if not force and self.last_send_time is not None:
                if now - self.last_send_time < self.action_gap_s:
                    return False

            target = self.active_chunk[self.index]
            self.index += 1

        self._send_packet(target)
        return True

    def _send_loop(self):
        while self.running.is_set():
            self.send_next()
            time.sleep(0.001)

    def _request_server_stop(self):
        if self.started:
            self.client.stop_arm_motion()
            self.started = False

    def finish(self):
        self.stop()

    def stop(self):
        stop_error = None
        try:
            self._request_server_stop()
        except Exception as exc:
            stop_error = exc
        finally:
            self.running.clear()
            if self.thread is not None:
                self.thread.join(timeout=1.0)
                self.thread = None

        if stop_error is not None:
            raise stop_error

    def close(self):
        if not self.closed:
            try:
                self.stop()
            finally:
                self.sock.close()
                self.closed = True
