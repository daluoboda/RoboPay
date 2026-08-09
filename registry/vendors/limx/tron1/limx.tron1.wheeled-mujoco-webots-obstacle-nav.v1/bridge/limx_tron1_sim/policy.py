"""Direct adapter for LimX's pinned WF_TRON1A Isaac Gym ONNX policy."""

from __future__ import annotations

import numpy as np

from .model import ENCODER_ONNX, POLICY_ONNX


JOINT_NAMES = (
    "abad_L_Joint",
    "hip_L_Joint",
    "knee_L_Joint",
    "wheel_L_Joint",
    "abad_R_Joint",
    "hip_R_Joint",
    "knee_R_Joint",
    "wheel_R_Joint",
)
NON_WHEEL = (0, 1, 2, 4, 5, 6)
STAND_TARGET = np.array([0.0, -0.9, 0.0, 0.0, 0.0, 0.9, 0.0, 0.0], dtype=np.float64)


def quaternion_matrix(quat_wxyz: np.ndarray) -> np.ndarray:
    w, x, y, z = (float(value) for value in quat_wxyz)
    norm = max(w * w + x * x + y * y + z * z, 1e-12)
    scale = 2.0 / norm
    return np.array(
        [
            [1 - scale * (y * y + z * z), scale * (x * y - z * w), scale * (x * z + y * w)],
            [scale * (x * y + z * w), 1 - scale * (x * x + z * z), scale * (y * z - x * w)],
            [scale * (x * z - y * w), scale * (y * z + x * w), 1 - scale * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


class LimXOnnxPolicy:
    """Inference and torque mapping matching LimX's published controller."""

    def __init__(self) -> None:
        import onnxruntime as ort

        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        providers = ["CPUExecutionProvider"]
        self.policy = ort.InferenceSession(str(POLICY_ONNX), sess_options=options, providers=providers)
        self.encoder = ort.InferenceSession(str(ENCODER_ONNX), sess_options=options, providers=providers)
        self.policy_input = self.policy.get_inputs()[0].name
        self.encoder_input = self.encoder.get_inputs()[0].name
        self.history = np.zeros(280, dtype=np.float32)
        self.last_actions = np.zeros(8, dtype=np.float64)
        self.initialized = False

    def _observation(
        self,
        q: np.ndarray,
        dq: np.ndarray,
        quat_wxyz: np.ndarray,
        gyro: np.ndarray,
    ) -> np.ndarray:
        rotation = quaternion_matrix(quat_wxyz)
        projected_gravity = rotation.T @ np.array([0.0, 0.0, -1.0])
        obs = np.concatenate(
            [
                np.asarray(gyro, dtype=np.float64) * 0.25,
                projected_gravity,
                np.asarray(q, dtype=np.float64)[list(NON_WHEEL)],
                np.asarray(dq, dtype=np.float64) * 0.05,
                self.last_actions,
            ]
        ).astype(np.float32)
        if not self.initialized:
            self.history = np.tile(obs, 10)
            self.initialized = True
        else:
            self.history[:-28] = self.history[28:]
            self.history[-28:] = obs
        return np.clip(obs, -100.0, 100.0)

    def actions(
        self,
        q: np.ndarray,
        dq: np.ndarray,
        quat_wxyz: np.ndarray,
        gyro: np.ndarray,
        command: tuple[float, float, float],
    ) -> np.ndarray:
        obs = self._observation(q, dq, quat_wxyz, gyro)
        encoded = self.encoder.run(None, {self.encoder_input: self.history.astype(np.float32)})[0].reshape(-1)
        scaled = np.asarray(command, dtype=np.float32) * np.array([1.5, 1.0, 0.5], dtype=np.float32)
        policy_input = np.concatenate([encoded, obs, scaled]).astype(np.float32)
        actions = self.policy.run(None, {self.policy_input: policy_input})[0].reshape(-1).astype(np.float64)
        self.last_actions = np.clip(actions, -100.0, 100.0)
        return self.last_actions.copy()

    @staticmethod
    def stand_torques(q: np.ndarray, dq: np.ndarray) -> np.ndarray:
        torques = np.zeros(8, dtype=np.float64)
        for index in NON_WHEEL:
            torques[index] = 42.0 * (STAND_TARGET[index] - q[index]) - 2.5 * dq[index]
        torques[3] = -0.8 * dq[3]
        torques[7] = -0.8 * dq[7]
        return np.clip(torques, [-80, -80, -80, -40, -80, -80, -80, -40], [80, 80, 80, 40, 80, 80, 80, 40])

    @staticmethod
    def action_torques(actions: np.ndarray, q: np.ndarray, dq: np.ndarray) -> np.ndarray:
        torques = np.zeros(8, dtype=np.float64)
        for index in NON_WHEEL:
            q_desired = 0.25 * actions[index]
            torques[index] = 42.0 * (q_desired - q[index]) - 2.5 * dq[index]
        for index in (3, 7):
            # The pinned Isaac Gym policy was trained with action_scale_vel=0.5.
            # The 0.8 value in the hardware deploy config is the wheel damping
            # gain, not the policy's velocity-action scale.
            velocity_desired = 0.5 * actions[index]
            torques[index] = 0.8 * (velocity_desired - dq[index])
        return np.clip(torques, [-80, -80, -80, -40, -80, -80, -80, -40], [80, 80, 80, 40, 80, 80, 80, 40])
