import logging
import socket
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger("ml_service")

PACKET_FORMAT = "27d2i"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)
DEFAULT_BUFFER_CAPACITY = 16
DEFAULT_HORIZON_PREV = 7
DEFAULT_SENSOR_SIZE = 29

ARM_STATE_MAP = {
    0: "IDLE",
    1: "MOVING",
    2: "ERROR",
}

GRIPPER_STATE_MAP = {
    0: "IDLE",
    1: "MOVING",
    2: "ERROR",
    3: "WIDTH_TOO_LARGE",
    4: "HOLDING",
    5: "OPEN_FAILED",
}


@dataclass
class Frame:
    """One decoded UDP state packet plus local receive metadata."""

    data: List[float]
    # Local receive time. Consumers use it to reject stale UDP data.
    timestamp: float
    # Local receive sequence. It detects whether this client saw a new frame;
    # it is not a robot-side packet id and cannot prove network packet loss.
    seq: int


class DataBuffer:
    """Small rolling UDP history buffer, intended for policy/VLA inference.

    For inference, prefer a small capacity such as 16 and read fixed-size
    history through get_padded_data_with_info(). Use a larger capacity only
    when an example needs to record many frames for plotting or debugging.
    """

    def __init__(
        self,
        capacity: int = DEFAULT_BUFFER_CAPACITY,
        horizon_prev: int = DEFAULT_HORIZON_PREV,
        sensor_size: int = DEFAULT_SENSOR_SIZE,
    ):
        """Create a bounded history buffer for decoded UDP frames."""

        if capacity < 1:
            raise ValueError("capacity must be >= 1")
        if horizon_prev < 1:
            raise ValueError("horizon_prev must be >= 1")
        if sensor_size < 1:
            raise ValueError("sensor_size must be >= 1")

        self.buffer = deque(maxlen=capacity)
        self.horizon_prev = horizon_prev
        self.sensor_size = sensor_size
        self._packet_seq = 0
        self.lock = threading.Lock()
        self.condition = threading.Condition(self.lock)


    def add_state(self, state: List[float]) -> Frame:
        """Store one decoded UDP state as a new Frame."""

        if len(state) < self.sensor_size:
            raise ValueError(
                f"Frame data length {len(state)} is less than sensor_size {self.sensor_size}"
            )

        now = time.monotonic()

        with self.condition:
            # DataBuffer is the single UDP state cache: every received packet
            # becomes one Frame used by both latest-state and history reads.
            self._packet_seq += 1
            frame = Frame(data=state, timestamp=now, seq=self._packet_seq)
            self.buffer.append(frame)
            self.condition.notify_all()
            return frame


    def get_padded_frames(self) -> List[Frame]:
        """Return the latest fixed-length history, padding if needed."""

        with self.lock:
            if not self.buffer:
                return []

            frames = list(self.buffer)
            if len(frames) >= self.horizon_prev:
                return frames[-self.horizon_prev:]

            # Keep ML/VLA input shape stable before the buffer is warm.
            pad_count = self.horizon_prev - len(frames)
            return [frames[0]] * pad_count + frames

    def get_latest_frame(self) -> Optional[Frame]:
        """Return the newest buffered Frame, or None if empty."""

        with self.lock:
            if not self.buffer:
                return None

            # Latest-state APIs intentionally read the newest DataBuffer frame
            # instead of maintaining a second shared latest-frame cache.
            return self.buffer[-1]


    def empty_buffer(self) -> None:
        """Clear buffered frames and reset the local receive sequence."""

        with self.condition:
            self.buffer.clear()
            self._packet_seq = 0

    def get_size(self) -> int:
        """Return the number of frames currently stored."""

        with self.lock:
            return len(self.buffer)


class UDPReceiver:
    """Receive UDP robot state and expose latest-state and history-window APIs."""

    def __init__(
        self,
        udp_ip: str,
        udp_port: int,
        capacity: int = DEFAULT_BUFFER_CAPACITY,
        horizon_prev: int = DEFAULT_HORIZON_PREV,
        sensor_size: int = DEFAULT_SENSOR_SIZE,
    ):
        """Bind a UDP socket and create the rolling DataBuffer."""

        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self._rxv = memoryview(bytearray(PACKET_SIZE))
        self._closed = False


        self.fresh_threshold_s = 0.2
        self.stale_threshold_s = 1.5
        self.expired_threshold_s = 3.0

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except OSError:
            pass
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self.sock.bind((udp_ip, udp_port))

        self._stop_event = threading.Event()
        self._receiver_thread: Optional[threading.Thread] = None
        self._sock_lock = threading.Lock()

        self.data_buffer = DataBuffer(
            capacity=capacity,
            horizon_prev=horizon_prev,
            sensor_size=sensor_size,
        )

    def _decode_packet(self, data: bytes) -> List[float]:
        """Decode one fixed-size UDP packet into 20 doubles plus 2 state ids."""

        if len(data) != PACKET_SIZE:
            raise ValueError(
                f"Received packet of incorrect size: expected {PACKET_SIZE}, got {len(data)}"
            )
        return list(struct.unpack(PACKET_FORMAT, data))

    def _classify_age(self, age: float) -> str:
        """Classify a frame age as fresh, stale, expired, or too_old."""

        if age < self.fresh_threshold_s:
            return "fresh"
        if age < self.stale_threshold_s:
            return "stale"
        if age < self.expired_threshold_s:
            return "expired"
        return "too_old"

    def _update_cache(self, state: List[float]) -> None:
        """Store a decoded state in the single DataBuffer cache."""

        # Store each valid UDP packet once. get_latest_frame_info() and
        # get_padded_data_with_info() both read from this same buffer.
        self.data_buffer.add_state(state)


    def _recv_one_packet(self, timeout: Optional[float]) -> Optional[List[float]]:
        """Receive, decode, and cache one UDP packet if available."""

        with self._sock_lock:
            previous_timeout = self.sock.gettimeout()
            try:
                self.sock.settimeout(timeout)
                try:
                    nbytes, _ = self.sock.recvfrom_into(self._rxv)
                except socket.timeout:
                    return None

                if nbytes != PACKET_SIZE:
                    raise ValueError(f"expected {PACKET_SIZE}, got {nbytes}")

                packet = self._rxv[:nbytes].tobytes()
            finally:
                self.sock.settimeout(previous_timeout)

        state = self._decode_packet(packet)
        self._update_cache(state)
        return state
        

    def get_latest_frame_info(
        self,
    ) -> Tuple[Optional[Dict[str, Any]], Optional[float], str, int]:
        """Return the newest buffered state and freshness metadata."""

        latest_frame = self.data_buffer.get_latest_frame()

        if latest_frame is None:
            return None, None, "empty", -1

        age = time.monotonic() - latest_frame.timestamp
        level = self._classify_age(age)

        arm_state_id = int(latest_frame.data[27])
        gripper_state_id = int(latest_frame.data[28])

        state = {
            "q": latest_frame.data[:7],
            "dq": latest_frame.data[7:14],
            "K_F_ext_hat": latest_frame.data[14:20],
            "tau_ext": latest_frame.data[20:27],
            "arm_state": ARM_STATE_MAP.get(arm_state_id, f"UNKNOWN({arm_state_id})"),
            "gripper_state": GRIPPER_STATE_MAP.get(
                gripper_state_id,
                f"UNKNOWN({gripper_state_id})",
            ),
            "arm_state_id": arm_state_id,
            "gripper_state_id": gripper_state_id,
        }

        return state, age, level, latest_frame.seq

    def get_padded_data_with_info(
        self,
        last_seen_seq: Optional[int] = None,
    ) -> Tuple[List[List[float]], Dict[str, Any]]:
        """Return a padded fixed-size history window for inference.

        This is the recommended UDP read path for VLA/policy inference because
        it returns a stable shape even when the buffer has fewer than
        horizon_prev frames.
        """

        frames = self.data_buffer.get_padded_frames()

        if not frames:
            return [], {
                "age": None,
                "level": "empty",
                "latest_seq": -1,
                "is_new": False,
                "num_frames": 0,
            }

        latest_frame = frames[-1]
        age = time.monotonic() - latest_frame.timestamp
        level = self._classify_age(age)

        stacked: List[float] = []
        for frame in frames:
            stacked.extend(frame.data[: self.data_buffer.sensor_size])

        meta = {
            "age": age,
            "level": level,
            "latest_seq": latest_frame.seq,
            "is_new": (last_seen_seq is None) or (latest_frame.seq != last_seen_seq),
            "num_frames": len(frames),
        }

        return [stacked], meta

    def _receive_loop(self, timeout: float = 0.1) -> None:
        """Background receive loop that continuously fills DataBuffer."""

        while not self._stop_event.is_set():
            try:
                self._recv_one_packet(timeout=timeout)
            except socket.timeout:
                continue
            except OSError:
                break
            except Exception as e:
                logger.warning("UDP receive loop error: %s", e)

    def start_receiving(self, timeout: float = 0.1) -> None:
        """Start the background UDP receive thread if it is not running."""

        if self._receiver_thread is not None and self._receiver_thread.is_alive():
            return

        self._stop_event.clear()
        self._receiver_thread = threading.Thread(
            target=self._receive_loop,
            args=(timeout,),
            daemon=True,
            name="UDPReceiverThread",
        )
        self._receiver_thread.start()

    def stop_receiving(self) -> None:
        """Stop the background UDP receive thread."""

        self._stop_event.set()
        if self._receiver_thread is not None:
            self._receiver_thread.join(timeout=1.0)
            self._receiver_thread = None

    def empty_buffer(self) -> None:
        """Flush socket backlog and clear buffered state."""

        self.flush()

    def clear_cache(self) -> None:
        """Clear buffered state without draining the socket."""

        self.data_buffer.empty_buffer()

    def drain_socket(self) -> int:
        """Drain pending UDP datagrams from the OS socket buffer."""

        drained = 0

        with self._sock_lock:
            previous_timeout = self.sock.gettimeout()

            try:
                self.sock.settimeout(0.0)

                while True:
                    try:
                        nbytes, _ = self.sock.recvfrom_into(self._rxv)
                    except (BlockingIOError, socket.timeout):
                        break
                    except OSError:
                        break

                    if nbytes == PACKET_SIZE:
                        drained += 1

            finally:
                try:
                    self.sock.settimeout(previous_timeout)
                except OSError:
                    pass

        return drained

    def flush(self) -> int:
        """Drain old UDP packets and reset the local DataBuffer."""

        drained = self.drain_socket()
        self.clear_cache()
        return drained




    def close(self) -> None:
        """Stop receiving and close the UDP socket."""

        if self._closed:
            return

        self._closed = True
        self.stop_receiving()

        with self._sock_lock:
            try:
                self.sock.close()
            except OSError:
                pass
