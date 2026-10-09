from pathlib import Path
import pinocchio as pin

from pinocchio.robot_wrapper import RobotWrapper

from typing import Union

import numpy as np

class RobotModel:
    """
    Pinocchio wrapper for the packaged Panda URDF.

    The default URDF lives inside the Python package. Mesh paths in that URDF
    are resolved relative to the package asset root, so package_dirs must point
    at the directory that contains the meshes folder.
    """

    def __init__(self, urdf_path=None):
        """Load the Panda URDF and initialize Pinocchio model/data."""

        self.urdf_path = self._resolve_urdf(urdf_path)

        self.robot = RobotWrapper.BuildFromURDF(
            str(self.urdf_path),
            package_dirs=[str(self.urdf_path.parent.parent)]
        )

        self.model = self.robot.model
        self.data = self.robot.data
        self.nq = self.model.nq
        self.nv = self.model.nv
 

    def _resolve_urdf(self, user_path):
        """Resolve a user-provided URDF path or the packaged default URDF."""

        if user_path:
            p = Path(user_path).expanduser()
            if not p.exists():
                raise FileNotFoundError(f"URDF not found: {p}")
            return p

        # Prefer the package-local URDF so installed users do not need a
        # separate franka_description checkout.
        default = Path(__file__).resolve().parent / "assets" / "panda" / "panda_arm.urdf"
        if default.exists():
            return default

        # Kept for old local installs that used the previous asset location.
        legacy_default = Path("~/.myframework/assets/franka/panda_arm.urdf").expanduser()
        if legacy_default.exists():
            return legacy_default


        if default.exists():
            return default

        raise RuntimeError(
            "URDF not found.\n"
            "Fix:\n"
            "  1. pass urdf_path=...\n"
            "  2. or run: python install.py"
        )
        
    def check_q(self, q: Union[np.ndarray, list, tuple]) -> np.ndarray:
        """Convert q to a flat float array and validate its length."""

        q_array = np.asarray(q, dtype=float).reshape(-1)
        if q_array.shape[0] != self.nq:
            raise ValueError(f"q has length {q_array.shape[0]}, expected {self.nq}")
        return q_array

    def check_dq(self, dq: Union[np.ndarray, list, tuple]) -> np.ndarray:
        """Convert dq to a flat float array and validate its length."""

        dq_array = np.asarray(dq, dtype=float).reshape(-1)
        if dq_array.shape[0] != self.nv:
            raise ValueError(f"dq has length {dq_array.shape[0]}, expected {self.nv}")
        return dq_array

    def list_frames(self):
        """Return all frame names available in the loaded URDF model."""

        return [f.name for f in self.model.frames]

    def list_joints(self):
        """Return all joint names available in the loaded URDF model."""

        return [j for j in self.model.names]


    def get_frame_id(self, frame_name: str) -> int:
        """Return the Pinocchio frame id for a named URDF frame."""

        if frame_name not in self.list_frames():
            raise ValueError(
                f"Frame '{frame_name}' not found.\n"
                f"Available: {self.list_frames()}"
            )
        return self.model.getFrameId(frame_name)


    def neutral_configuration(self):
        """Return Pinocchio's neutral configuration for this model."""

        return pin.neutral(self.model)


    def forward_kinematics(self, q):
        """Update Pinocchio placements for the given joint configuration."""

        q = self.check_q(q)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

    def get_frame_pose(self, q, frame_name="panda_hand"):
        """Return translation, rotation, and 4x4 pose for a URDF frame."""

        self.forward_kinematics(q)

        if frame_name not in self.list_frames():
            # Keep older examples alive if a requested hand/TCP frame is absent.
            # New code should normally use RemoteControllerClient.default_frame_name.
            frame_name = "panda_link8"

        frame_id = self.get_frame_id(frame_name)
        placement = self.data.oMf[frame_id]

        T = np.eye(4)
        T[:3, :3] = placement.rotation
        T[:3, 3] = placement.translation

        return {
            "translation": placement.translation.copy(),
            "rotation": placement.rotation.copy(),
            "T": T,
        }

    def get_frame_jacobian(self, q, frame_name="panda_hand"):
        """Return the 6xN frame Jacobian in LOCAL_WORLD_ALIGNED coordinates."""

        q = self.check_q(q)

        if frame_name not in self.list_frames():
            # Match get_frame_pose() fallback so pose/Jacobian are computed at
            # the same frame when a legacy frame name is missing.
            frame_name = "panda_link8"

        frame_id = self.get_frame_id(frame_name)

        J = pin.computeFrameJacobian(
            self.model,
            self.data,
            q,
            frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )
        # LOCAL_WORLD_ALIGNED expresses angular/linear axes in the world/base
        # orientation while keeping the frame point at the requested TCP.
        return J.copy()

    def get_mass_matrix(self, q):
        """Return the joint-space mass matrix M(q)."""

        q = self.check_q(q)
        M = pin.crba(self.model, self.data, q)
        return 0.5 * (M + M.T)

    def get_gravity(self, q):
        """Return the generalized gravity vector g(q)."""

        q = self.check_q(q)
        return pin.computeGeneralizedGravity(self.model, self.data, q).copy()

    def get_coriolis(self, q, dq=None):
        """Return the velocity-dependent generalized term c(q, dq)."""

        q = self.check_q(q)

        if dq is None:
            dq = np.zeros(self.nv)

        dq = self.check_dq(dq)

        ddq = np.zeros(self.nv)

        tau = pin.rnea(self.model, self.data, q, dq, ddq)
        g = pin.computeGeneralizedGravity(self.model, self.data, q)

        return (tau - g).copy()

    def get_all_basic_terms(self, q, dq=None, frame_name="panda_hand"):
        """Return pose, Jacobian, mass matrix, gravity, and coriolis terms."""

        q = self.check_q(q)

        if dq is None:
            dq = np.zeros(self.nv)
        else:
            dq = self.check_dq(dq)

        pose = self.get_frame_pose(q, frame_name)
        J = self.get_frame_jacobian(q, frame_name)
        M = self.get_mass_matrix(q)
        g = self.get_gravity(q)
        c = self.get_coriolis(q, dq)

        return {
            "q": q,
            "dq": dq,
            "ee_translation": pose["translation"],
            "ee_rotation": pose["rotation"],
            "ee_T": pose["T"],
            "J": J,
            "M": M,
            "g": g,
            "c": c,
        }
