import argparse
import bisect
import csv
import json
import math
import re
import socket
import struct
import sys
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

try:
    import pyqtgraph.opengl as gl
    from PyQt6.QtCore import QThread, QUrl, pyqtSignal, Qt
    from PyQt6.QtGui import QDesktopServices, QPixmap
    from PyQt6.QtWidgets import (
        QApplication,
        QFileDialog,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QMainWindow,
        QPushButton,
        QScrollArea,
        QVBoxLayout,
        QWidget,
    )
    GUI_DEPENDENCIES_AVAILABLE = True
except ImportError:
    GUI_DEPENDENCIES_AVAILABLE = False

    class _MissingGuiBase:
        def __init__(self, *args, **kwargs):
            raise ImportError("GUI mode requires PyQt6 and pyqtgraph. Install requirements_visualizer.txt first.")

    class _DummyQtNamespace:
        AlignTop = 0

    class _DummyGLModule:
        GLViewWidget = _MissingGuiBase
        GLLinePlotItem = _MissingGuiBase
        GLScatterPlotItem = _MissingGuiBase
        GLGridItem = _MissingGuiBase
        GLAxisItem = _MissingGuiBase

    def pyqtSignal(*args, **kwargs):
        return None

    gl = _DummyGLModule()
    QThread = _MissingGuiBase
    Qt = _DummyQtNamespace()
    QApplication = _MissingGuiBase
    QDesktopServices = _MissingGuiBase
    QFileDialog = _MissingGuiBase
    QGroupBox = _MissingGuiBase
    QHBoxLayout = _MissingGuiBase
    QLabel = _MissingGuiBase
    QMainWindow = _MissingGuiBase
    QPixmap = _MissingGuiBase
    QPushButton = _MissingGuiBase
    QScrollArea = _MissingGuiBase
    QVBoxLayout = _MissingGuiBase
    QUrl = _MissingGuiBase
    QWidget = _MissingGuiBase

try:
    import cv2
except ImportError:
    cv2 = None

try:
    from scipy.signal import correlate, correlation_lags, savgol_filter
except ImportError:
    correlate = None
    correlation_lags = None
    savgol_filter = None

try:
    from scipy.optimize import least_squares, minimize_scalar
except ImportError:
    least_squares = None
    minimize_scalar = None

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    plt = None


FLOAT32_PACKET = struct.Struct("<Id7f")
V2_PACKET = struct.Struct("<4sHHIdd7fI")
V2_MAGIC = b"APS2"


@dataclass(frozen=True)
class PoseSample:
    stream: str
    sequence: int
    sender_time: float
    recv_time: float
    position: np.ndarray
    quaternion: np.ndarray
    sensor_time: Optional[float] = None
    protocol_version: int = 1
    checksum_valid: Optional[bool] = None


def decode_packet(packet: bytes):
    if len(packet) == V2_PACKET.size and packet[:4] == V2_MAGIC:
        return decode_v2_packet(packet)

    if len(packet) != FLOAT32_PACKET.size:
        raise ValueError(f"Expected {FLOAT32_PACKET.size} bytes, got {len(packet)}")
    sequence, sender_time, x, y, z, qx, qy, qz, qw = FLOAT32_PACKET.unpack(packet)
    quat = normalize_quaternion(np.array([qx, qy, qz, qw], dtype=float))
    return sequence, sender_time, np.array([x, y, z], dtype=float), quat, None, 1, None


def decode_v2_packet(packet: bytes):
    payload = packet[:-4]
    checksum = struct.unpack("<I", packet[-4:])[0]
    checksum_valid = fnv1a_bytes(payload) == checksum
    magic, version, flags, sequence, sensor_time, received_time, x, y, z, qx, qy, qz, qw, _ = V2_PACKET.unpack(packet)
    if magic != V2_MAGIC:
        raise ValueError("Invalid APS2 packet magic")
    if not checksum_valid:
        raise ValueError("Invalid APS2 packet checksum")

    has_sensor_time = bool(flags & 1)
    quat = normalize_quaternion(np.array([qx, qy, qz, qw], dtype=float))
    return sequence, received_time, np.array([x, y, z], dtype=float), quat, sensor_time if has_sensor_time else None, version, checksum_valid


def fnv1a_bytes(payload):
    value = 2166136261
    for byte in payload:
        value ^= byte
        value = (value * 16777619) & 0xFFFFFFFF
    return value


def normalize_quaternion(quaternion):
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-9:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return quaternion / norm


def quaternion_to_matrix(quaternion):
    x, y, z, w = normalize_quaternion(quaternion)
    return np.array(
        [
            [1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
            [2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w],
            [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y],
        ],
        dtype=float,
    )


def quaternion_angle_error_degrees(first, second):
    dot = abs(float(np.dot(normalize_quaternion(first), normalize_quaternion(second))))
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


def quaternion_multiply(first, second):
    ax, ay, az, aw = first
    bx, by, bz, bw = second
    return normalize_quaternion(
        np.array(
            [
                aw * bx + ax * bw + ay * bz - az * by,
                aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw,
                aw * bw - ax * bx - ay * by - az * bz,
            ],
            dtype=float,
        )
    )


def quaternion_conjugate(quaternion):
    x, y, z, w = normalize_quaternion(quaternion)
    return np.array([-x, -y, -z, w], dtype=float)


def matrix_to_quaternion(matrix):
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (matrix[2, 1] - matrix[1, 2]) / scale
        qy = (matrix[0, 2] - matrix[2, 0]) / scale
        qz = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        diagonal = np.diag(matrix)
        axis = int(np.argmax(diagonal))
        if axis == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            qw = (matrix[2, 1] - matrix[1, 2]) / scale
            qx = 0.25 * scale
            qy = (matrix[0, 1] + matrix[1, 0]) / scale
            qz = (matrix[0, 2] + matrix[2, 0]) / scale
        elif axis == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            qw = (matrix[0, 2] - matrix[2, 0]) / scale
            qx = (matrix[0, 1] + matrix[1, 0]) / scale
            qy = 0.25 * scale
            qz = (matrix[1, 2] + matrix[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            qw = (matrix[1, 0] - matrix[0, 1]) / scale
            qx = (matrix[0, 2] + matrix[2, 0]) / scale
            qy = (matrix[1, 2] + matrix[2, 1]) / scale
            qz = 0.25 * scale
    return normalize_quaternion(np.array([qx, qy, qz, qw], dtype=float))


def average_quaternion(quaternions):
    if not quaternions:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)

    reference = normalize_quaternion(quaternions[0])
    accumulator = np.zeros(4, dtype=float)
    for quaternion in quaternions:
        aligned = normalize_quaternion(quaternion)
        if np.dot(reference, aligned) < 0.0:
            aligned = -aligned
        accumulator += aligned
    return normalize_quaternion(accumulator)


def nearest_by_sender_time(samples, target_time, max_delta_seconds):
    best = None
    best_delta = float("inf")
    for sample in samples:
        delta = abs(sample.sender_time - target_time)
        if delta < best_delta:
            best = sample
            best_delta = delta
    if best is None or best_delta > max_delta_seconds:
        return None, best_delta
    return best, best_delta


def load_pose_csv(path, stream):
    samples = []
    with Path(path).open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for index, row in enumerate(reader):
            try:
                sequence = int(float(row.get("sequence", index)))
                # Prefer relative_time for synchronization, fall back to absolute timestamps
                sender_time = first_float(row, ["relative_time", "time", "sender_time", "received_time", "timestamp"])
                frame_time = first_float(row, ["frame_time", "received_time", "recv_time"], default=sender_time)
                sensor_time = first_float(row, ["sensor_time"], default=None, required=False)
                position = np.array([
                    float(row["x"]),
                    float(row["y"]),
                    float(row["z"]),
                ], dtype=float)
                quaternion = normalize_quaternion(np.array([
                    float(row["qx"]),
                    float(row["qy"]),
                    float(row["qz"]),
                    float(row["qw"]),
                ], dtype=float))
            except (KeyError, TypeError, ValueError):
                continue

            samples.append(
                PoseSample(
                    stream=stream,
                    sequence=sequence,
                    sender_time=sender_time,
                    recv_time=frame_time,
                    position=position,
                    quaternion=quaternion,
                    sensor_time=sensor_time,
                    protocol_version=int(float(row.get("protocol_version", 1) or 1)),
                    checksum_valid=parse_bool(row.get("checksum_valid")),
                )
            )
    return samples


def first_float(row, keys, default=None, required=True):
    for key in keys:
        value = row.get(key)
        if value not in (None, ""):
            return float(value)
    if required and default is None:
        raise ValueError(f"Missing required numeric field from {keys}")
    return default


def parse_bool(value):
    if value is None or value == "":
        return None
    return str(value).strip().lower() in {"1", "true", "yes"}


def compose_transform(rotation, translation):
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = np.asarray(rotation, dtype=float)
    transform[:3, 3] = np.asarray(translation, dtype=float).reshape(3)
    return transform


def pose_to_matrix(position, quaternion, translation_scale=1.0):
    return compose_transform(
        quaternion_to_matrix(quaternion),
        np.asarray(position, dtype=float) * float(translation_scale),
    )


def invert_transform(transform):
    rotation = np.asarray(transform[:3, :3], dtype=float)
    translation = np.asarray(transform[:3, 3], dtype=float)
    inverse = np.eye(4, dtype=float)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ translation
    return inverse


def relative_transform(first, second):
    return invert_transform(first) @ second


def smooth_signal(values, preferred_window=11, polyorder=2):
    if savgol_filter is None:
        return np.asarray(values, dtype=float)

    data = np.asarray(values, dtype=float)
    if data.size < 5:
        return data

    window = min(int(preferred_window), int(data.size))
    if window % 2 == 0:
        window -= 1
    if window <= polyorder:
        return data
    return savgol_filter(data, window_length=window, polyorder=min(polyorder, window - 1), mode="interp")


def average_transform(transforms):
    if not transforms:
        raise ValueError("At least one transform is required for averaging.")

    translations = np.array([transform[:3, 3] for transform in transforms], dtype=float)
    quaternions = [matrix_to_quaternion(transform[:3, :3]) for transform in transforms]
    return compose_transform(
        quaternion_to_matrix(average_quaternion(quaternions)),
        np.mean(translations, axis=0),
    )


def rotation_matrix_to_rotvec(rotation):
    if cv2 is None:
        raise ImportError("OpenCV is required for Rodrigues rotation conversions.")
    rotvec, _ = cv2.Rodrigues(np.asarray(rotation, dtype=float))
    return rotvec.reshape(3)


def rotvec_to_rotation_matrix(rotvec):
    if cv2 is None:
        raise ImportError("OpenCV is required for Rodrigues rotation conversions.")
    rotation, _ = cv2.Rodrigues(np.asarray(rotvec, dtype=float).reshape(3, 1))
    return rotation


def slugify_for_path(value):
    text = re.sub(r"[^A-Za-z0-9._-]+", "_", str(value))
    text = text.strip("._-")
    return text or "run"


def prepare_run_output_dir(base_output_dir, arkit_csv, sensor_csv):
    base_dir = Path(base_output_dir)
    base_dir.mkdir(parents=True, exist_ok=True)
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    arkit_stem = slugify_for_path(Path(arkit_csv).stem)
    sensor_stem = slugify_for_path(Path(sensor_csv).stem)
    run_dir = base_dir / f"{timestamp}_{arkit_stem}__{sensor_stem}"
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir


ERROR_PLOT_FILES = [
    ("3D Trajectory Overlap", "trajectory_overlap_3d.png"),
    ("XYZ Error Curve", "axis_error_curves.png"),
    ("Position Error Distribution", "absolute_error_histogram.png"),
    ("Motion Error Distribution", "relative_error_histogram.png"),
]

LEGACY_DIAGNOSTIC_PLOT_FILES = [
    ("Position Error Before/After", "absolute_error_comparison.png"),
    ("XYZ Error Before/After", "axis_error_comparison.png"),
    ("Calibration Process", "absolute_error_stages.png"),
]

ALL_KNOWN_PLOT_FILES = ERROR_PLOT_FILES + LEGACY_DIAGNOSTIC_PLOT_FILES


def find_latest_error_plot_dir(base_output_dir="offline_calibration_output"):
    base_dir = Path(base_output_dir)
    if not base_dir.exists():
        return None

    candidates = []
    for path in base_dir.iterdir():
        if not path.is_dir():
            continue
        if any((path / filename).exists() for _, filename in ALL_KNOWN_PLOT_FILES):
            candidates.append(path)

    if not candidates:
        return None

    return max(candidates, key=lambda path: path.stat().st_mtime)


class SimilarityTransform:
    def __init__(self, scale=1.0, rotation=None, translation=None, orientation_delta=None):
        self.scale = scale
        self.rotation = rotation if rotation is not None else np.eye(3, dtype=float)
        self.translation = translation if translation is not None else np.zeros(3, dtype=float)
        self.orientation_delta = orientation_delta if orientation_delta is not None else np.array([0.0, 0.0, 0.0, 1.0])

    def apply_position(self, position):
        return self.scale * (self.rotation @ position) + self.translation

    def apply_quaternion(self, quaternion):
        rotated = quaternion_multiply(matrix_to_quaternion(self.rotation), quaternion)
        return quaternion_multiply(self.orientation_delta, rotated)

    def to_dict(self):
        return {
            "scale": self.scale,
            "rotation": self.rotation.tolist(),
            "translation": self.translation.tolist(),
            "orientation_delta": self.orientation_delta.tolist(),
        }

    @classmethod
    def from_dict(cls, data):
        return cls(
            scale=float(data.get("scale", 1.0)),
            rotation=np.array(data.get("rotation", np.eye(3).tolist()), dtype=float),
            translation=np.array(data.get("translation", [0.0, 0.0, 0.0]), dtype=float),
            orientation_delta=np.array(data.get("orientation_delta", [0.0, 0.0, 0.0, 1.0]), dtype=float),
        )


class CalibrationResult:
    def __init__(self):
        self.enabled = False
        self.transform = SimilarityTransform()
        self.time_offset = 0.0
        self.time_slope = 1.0
        self.mean_time_error = None
        self.time_rmse = None
        self.max_time_error = None
        self.position_rmse = None
        self.angle_rmse = None
        self.pair_count = 0
        self.scale = 1.0
        self.quality = "waiting"
        self.motion_coverage = "none"
        self.inlier_ratio = 0.0

    def sensor_to_arkit_time(self, sensor_time):
        return self.time_slope * sensor_time + self.time_offset

    def arkit_to_sensor_time(self, arkit_time):
        if abs(self.time_slope) < 1e-9:
            return arkit_time - self.time_offset
        return (arkit_time - self.time_offset) / self.time_slope

    def to_dict(self):
        return {
            "enabled": self.enabled,
            "transform": self.transform.to_dict(),
            "time_offset": self.time_offset,
            "time_slope": self.time_slope,
            "mean_time_error": self.mean_time_error,
            "time_rmse": self.time_rmse,
            "max_time_error": self.max_time_error,
            "position_rmse": self.position_rmse,
            "angle_rmse": self.angle_rmse,
            "pair_count": self.pair_count,
            "scale": self.scale,
            "quality": self.quality,
            "motion_coverage": self.motion_coverage,
            "inlier_ratio": self.inlier_ratio,
        }

    @classmethod
    def from_dict(cls, data):
        result = cls()
        result.enabled = bool(data.get("enabled", True))
        result.transform = SimilarityTransform.from_dict(data.get("transform", {}))
        result.time_offset = float(data.get("time_offset", 0.0))
        result.time_slope = float(data.get("time_slope", 1.0))
        result.mean_time_error = data.get("mean_time_error")
        result.time_rmse = data.get("time_rmse")
        result.max_time_error = data.get("max_time_error")
        result.position_rmse = data.get("position_rmse")
        result.angle_rmse = data.get("angle_rmse")
        result.pair_count = int(data.get("pair_count", 0))
        result.scale = float(data.get("scale", result.transform.scale))
        result.quality = data.get("quality", "loaded")
        result.motion_coverage = data.get("motion_coverage", "loaded")
        result.inlier_ratio = float(data.get("inlier_ratio", 1.0))
        return result


@dataclass(frozen=True)
class TrajectorySeries:
    name: str
    samples: list[PoseSample]
    time: np.ndarray
    position: np.ndarray
    quaternion: np.ndarray

    @classmethod
    def from_samples(cls, name, samples):
        ordered = sorted(samples, key=lambda sample: sample.sender_time)
        time_values = np.array([sample.sender_time for sample in ordered], dtype=float)
        if time_values.size == 0:
            raise ValueError(f"{name} data is empty.")
        time_values = time_values - float(np.min(time_values))
        positions = np.array([sample.position for sample in ordered], dtype=float)
        quaternions = np.array([normalize_quaternion(sample.quaternion) for sample in ordered], dtype=float)
        return cls(name=name, samples=ordered, time=time_values, position=positions, quaternion=quaternions)


@dataclass(frozen=True)
class VelocityProfile:
    time: np.ndarray
    speed: np.ndarray
    label: str


@dataclass(frozen=True)
class TimeSyncResult:
    time_shift: float
    arkit_time_shifted: np.ndarray
    resample_time: np.ndarray
    robot_speed_interp: np.ndarray
    arkit_speed_interp: np.ndarray
    correlation: np.ndarray
    lag_samples: np.ndarray
    dt: float
    robot_profile: VelocityProfile
    arkit_profile: VelocityProfile


@dataclass(frozen=True)
class ScaleCalibrationResult:
    scale_factor: float
    initial_scale_factor: float
    max_v_robot: float
    max_v_arkit: float
    arkit_scaled_profile: VelocityProfile


@dataclass(frozen=True)
class MatchedFramePair:
    robot_index: int
    arkit_index: int
    time_delta: float
    robot_time: float
    arkit_time_shifted: float
    T_base_gripper: np.ndarray
    T_world_cam: np.ndarray
    T_target_cam: np.ndarray


@dataclass(frozen=True)
class HandEyeResult:
    R_cam2gripper: np.ndarray
    t_cam2gripper: np.ndarray
    T_cam2gripper: np.ndarray
    T_base_world: np.ndarray
    matched_pairs: list[MatchedFramePair]


@dataclass(frozen=True)
class EvaluationResult:
    relative_errors_mm: np.ndarray
    mean_relative_error_mm: float
    max_relative_error_mm: float
    absolute_errors_mm: np.ndarray
    axis_errors_mm: np.ndarray
    mean_absolute_error_mm: float
    rmse_absolute_error_mm: float
    max_absolute_error_mm: float
    predicted_camera_positions: np.ndarray
    predicted_gripper_positions: np.ndarray
    robot_positions: np.ndarray
    pair_time_deltas_ms: np.ndarray


@dataclass(frozen=True)
class StageEvaluationResult:
    name: str
    absolute_errors_mm: np.ndarray
    mean_absolute_error_mm: float
    rmse_absolute_error_mm: float
    max_absolute_error_mm: float


@dataclass(frozen=True)
class CrossValidationFoldResult:
    fold_index: int
    best_scale: float
    train_best_abs_mean_mm: float
    val_best_abs_mean_mm: float
    train_best_rel_mean_mm: float
    val_best_rel_mean_mm: float
    train_init_abs_mean_mm: float
    val_init_abs_mean_mm: float
    train_init_rel_mean_mm: float
    val_init_rel_mean_mm: float
    best_abs_gap_mm: float
    init_abs_gap_mm: float


@dataclass(frozen=True)
class CrossValidationSummary:
    fold_count: int
    best_scale_mean: float
    best_scale_std: float
    train_best_abs_mean_mm: float
    val_best_abs_mean_mm: float
    best_abs_gap_mean_mm: float
    train_init_abs_mean_mm: float
    val_init_abs_mean_mm: float
    init_abs_gap_mean_mm: float
    val_abs_improvement_mm: float
    folds: list[CrossValidationFoldResult]


@dataclass(frozen=True)
class OfflineCalibrationResult:
    time_sync: TimeSyncResult
    scale: ScaleCalibrationResult
    initial_hand_eye: HandEyeResult
    initial_evaluation: EvaluationResult
    hand_eye: HandEyeResult
    evaluation: EvaluationResult
    stage_evaluations: list[StageEvaluationResult]
    cross_validation: Optional[CrossValidationSummary] = None

    @property
    def time_shift(self):
        return self.time_sync.time_shift

    @property
    def scale_factor(self):
        return self.scale.scale_factor


class PoseDataLoader:
    def load_series(self, path, stream_name):
        samples = load_pose_csv(path, stream_name)
        if len(samples) < 3:
            raise ValueError(f"{stream_name} requires at least 3 valid samples, got {len(samples)}.")
        return TrajectorySeries.from_samples(stream_name, samples)


class TimeSynchronizer:
    def __init__(self, smooth_window=11, active_motion_ratio=0.05, local_refine_range=0.35, local_refine_step=0.002):
        self.smooth_window = smooth_window
        self.active_motion_ratio = active_motion_ratio
        self.local_refine_range = local_refine_range
        self.local_refine_step = local_refine_step

    def compute_velocity_profile(self, series, label):
        if len(series.time) < 3:
            raise ValueError(f"{label} requires at least 3 samples to estimate velocity.")
        velocity = np.gradient(series.position, series.time, axis=0, edge_order=2)
        speed = np.linalg.norm(velocity, axis=1)
        speed = smooth_signal(speed, preferred_window=self.smooth_window)
        return VelocityProfile(time=series.time.copy(), speed=speed, label=label)

    def synchronize(self, robot_series, arkit_series):
        if correlate is None or correlation_lags is None:
            raise ImportError("scipy is required for cross-correlation time synchronization. Install scipy first.")

        robot_profile = self.compute_velocity_profile(robot_series, "Robot speed")
        arkit_profile = self.compute_velocity_profile(arkit_series, "ARKit speed")
        dt = self.estimate_common_dt(robot_profile.time, arkit_profile.time)

        coarse_shift, correlation, lag_samples = self.estimate_shift_from_active_windows(robot_profile, arkit_profile, dt)
        time_shift = self.refine_shift_locally(robot_profile, arkit_profile, coarse_shift)
        arkit_time_shifted = arkit_series.time + time_shift

        overlap_start = max(float(robot_profile.time[0]), float(np.min(arkit_time_shifted)))
        overlap_end = min(float(robot_profile.time[-1]), float(np.max(arkit_time_shifted)))
        if overlap_end <= overlap_start:
            raise ValueError("Time range is too short for synchronization after applying the estimated shift.")

        resample_time = np.arange(overlap_start, overlap_end + 0.5 * dt, dt, dtype=float)
        robot_interp = np.interp(resample_time, robot_profile.time, robot_profile.speed)
        arkit_interp = np.interp(resample_time, arkit_time_shifted, arkit_profile.speed)

        return TimeSyncResult(
            time_shift=time_shift,
            arkit_time_shifted=arkit_time_shifted,
            resample_time=resample_time,
            robot_speed_interp=robot_interp,
            arkit_speed_interp=arkit_interp,
            correlation=correlation,
            lag_samples=lag_samples,
            dt=dt,
            robot_profile=robot_profile,
            arkit_profile=arkit_profile,
        )

    def estimate_shift_from_active_windows(self, robot_profile, arkit_profile, dt):
        robot_start, robot_end = self.active_window(robot_profile)
        arkit_start, arkit_end = self.active_window(arkit_profile)

        robot_time = np.arange(robot_start, robot_end + 0.5 * dt, dt, dtype=float)
        arkit_time = np.arange(arkit_start, arkit_end + 0.5 * dt, dt, dtype=float)
        robot_interp = np.interp(robot_time, robot_profile.time, robot_profile.speed)
        arkit_interp = np.interp(arkit_time, arkit_profile.time, arkit_profile.speed)
        robot_zero_mean = robot_interp - np.mean(robot_interp)
        arkit_zero_mean = arkit_interp - np.mean(arkit_interp)

        correlation = correlate(robot_zero_mean, arkit_zero_mean, mode="full", method="fft")
        lag_samples = correlation_lags(len(robot_zero_mean), len(arkit_zero_mean), mode="full")
        best_index = int(np.argmax(correlation))
        coarse_shift = (float(robot_time[0]) - float(arkit_time[0])) + float(lag_samples[best_index]) * dt
        return coarse_shift, correlation, lag_samples

    def active_window(self, profile):
        threshold = self.active_motion_ratio * float(np.max(profile.speed))
        active_indices = np.flatnonzero(profile.speed >= threshold)
        if active_indices.size == 0:
            return float(profile.time[0]), float(profile.time[-1])
        return float(profile.time[active_indices[0]]), float(profile.time[active_indices[-1]])

    def refine_shift_locally(self, robot_profile, arkit_profile, initial_shift):
        best_shift = float(initial_shift)
        best_score = float("inf")
        candidate_shifts = np.arange(
            initial_shift - self.local_refine_range,
            initial_shift + self.local_refine_range + 0.5 * self.local_refine_step,
            self.local_refine_step,
            dtype=float,
        )

        for shift in candidate_shifts:
            score = self.shift_alignment_score(robot_profile, arkit_profile, shift)
            if score < best_score:
                best_score = score
                best_shift = float(shift)
        return best_shift

    def shift_alignment_score(self, robot_profile, arkit_profile, shift):
        shifted_time = arkit_profile.time + float(shift)
        overlap_start = max(float(robot_profile.time[0]), float(np.min(shifted_time)))
        overlap_end = min(float(robot_profile.time[-1]), float(np.max(shifted_time)))
        if overlap_end <= overlap_start:
            return float("inf")

        dt = self.estimate_common_dt(robot_profile.time, shifted_time)
        sample_time = np.arange(overlap_start, overlap_end + 0.5 * dt, dt, dtype=float)
        robot_speed = np.interp(sample_time, robot_profile.time, robot_profile.speed)
        arkit_speed = np.interp(sample_time, shifted_time, arkit_profile.speed)

        active_threshold = self.active_motion_ratio * max(float(np.max(robot_profile.speed)), float(np.max(arkit_profile.speed)))
        active_mask = (robot_speed >= active_threshold) | (arkit_speed >= active_threshold)
        if np.count_nonzero(active_mask) < 10:
            return float("inf")

        robot_active = robot_speed[active_mask]
        arkit_active = arkit_speed[active_mask]
        peak = float(np.max(arkit_active))
        if peak <= 1e-9:
            return float("inf")

        scale = float(np.max(robot_active)) / peak
        residual = robot_active - arkit_active * scale
        return float(np.mean(residual * residual))

    @staticmethod
    def estimate_common_dt(first_time, second_time):
        first_dt = np.diff(first_time)
        second_dt = np.diff(second_time)
        positive = np.concatenate([first_dt[first_dt > 1e-9], second_dt[second_dt > 1e-9]])
        if positive.size == 0:
            raise ValueError("Unable to estimate sampling interval from timestamps.")
        return float(np.median(positive))


class ScaleCalibrator:
    def calibrate(self, robot_profile, arkit_profile, arkit_time_shifted):
        overlap_start = max(float(robot_profile.time[0]), float(np.min(arkit_time_shifted)))
        overlap_end = min(float(robot_profile.time[-1]), float(np.max(arkit_time_shifted)))
        if overlap_end <= overlap_start:
            raise ValueError("No overlapping time span remains after time synchronization.")

        dt = TimeSynchronizer.estimate_common_dt(robot_profile.time, arkit_time_shifted)
        overlap_time = np.arange(overlap_start, overlap_end + 0.5 * dt, dt, dtype=float)
        robot_speed = np.interp(overlap_time, robot_profile.time, robot_profile.speed)
        arkit_speed = np.interp(overlap_time, arkit_time_shifted, arkit_profile.speed)

        max_v_robot = float(np.max(robot_speed))
        max_v_arkit = float(np.max(arkit_speed))
        return ScaleCalibrationResult(
            scale_factor=1.0,
            initial_scale_factor=1.0,
            max_v_robot=max_v_robot,
            max_v_arkit=max_v_arkit,
            arkit_scaled_profile=VelocityProfile(
                time=overlap_time,
                speed=arkit_speed,
                label="ARKit speed (time-aligned, scale fixed at 1)",
            ),
        )


class HandEyeCalibrator:
    def __init__(self, max_pair_delta=0.05):
        self.max_pair_delta = max_pair_delta

    def calibrate(self, robot_series, arkit_series, arkit_time_shifted, scale_factor):
        if cv2 is None:
            raise ImportError("opencv-python or opencv-python-headless is required for hand-eye calibration.")

        pairs = self.match_frames(robot_series, arkit_series, arkit_time_shifted, scale_factor)
        if len(pairs) < 3:
            raise ValueError(f"Hand-eye calibration needs at least 3 matched poses, got {len(pairs)}.")

        R_gripper2base = [pair.T_base_gripper[:3, :3] for pair in pairs]
        t_gripper2base = [pair.T_base_gripper[:3, 3].reshape(3, 1) for pair in pairs]
        R_target2cam = [pair.T_target_cam[:3, :3] for pair in pairs]
        t_target2cam = [pair.T_target_cam[:3, 3].reshape(3, 1) for pair in pairs]

        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
            R_gripper2base=R_gripper2base,
            t_gripper2base=t_gripper2base,
            R_target2cam=R_target2cam,
            t_target2cam=t_target2cam,
            method=cv2.CALIB_HAND_EYE_TSAI,
        )

        R_cam2gripper = np.asarray(R_cam2gripper, dtype=float)
        t_cam2gripper = np.asarray(t_cam2gripper, dtype=float).reshape(3)
        T_cam2gripper = compose_transform(R_cam2gripper, t_cam2gripper)

        T_base_world_candidates = [
            pair.T_base_gripper @ T_cam2gripper @ pair.T_target_cam
            for pair in pairs
        ]
        T_base_world = average_transform(T_base_world_candidates)

        return HandEyeResult(
            R_cam2gripper=R_cam2gripper,
            t_cam2gripper=t_cam2gripper,
            T_cam2gripper=T_cam2gripper,
            T_base_world=T_base_world,
            matched_pairs=pairs,
        )

    def match_frames(self, robot_series, arkit_series, arkit_time_shifted, scale_factor):
        arkit_time_shifted = np.asarray(arkit_time_shifted, dtype=float)
        valid_indices = np.argsort(arkit_time_shifted)
        arkit_times_sorted = arkit_time_shifted[valid_indices]
        overlap_start = max(float(robot_series.time[0]), float(arkit_times_sorted[0]))
        overlap_end = min(float(robot_series.time[-1]), float(arkit_times_sorted[-1]))
        if overlap_end <= overlap_start:
            raise ValueError("No overlapping time span available for nearest-neighbor pose pairing.")

        pairs = []
        for robot_index, robot_time in enumerate(robot_series.time):
            if robot_time < overlap_start or robot_time > overlap_end:
                continue

            insert_at = int(np.searchsorted(arkit_times_sorted, robot_time))
            candidate_positions = []
            if insert_at < len(arkit_times_sorted):
                candidate_positions.append(insert_at)
            if insert_at > 0:
                candidate_positions.append(insert_at - 1)
            if not candidate_positions:
                continue

            best_sorted_idx = min(candidate_positions, key=lambda idx: abs(arkit_times_sorted[idx] - robot_time))
            arkit_index = int(valid_indices[best_sorted_idx])
            delta = float(abs(arkit_time_shifted[arkit_index] - robot_time))
            if delta > self.max_pair_delta:
                continue

            T_base_gripper = pose_to_matrix(robot_series.position[robot_index], robot_series.quaternion[robot_index])
            T_world_cam = pose_to_matrix(
                arkit_series.position[arkit_index],
                arkit_series.quaternion[arkit_index],
                translation_scale=scale_factor,
            )
            pairs.append(
                MatchedFramePair(
                    robot_index=robot_index,
                    arkit_index=arkit_index,
                    time_delta=delta,
                    robot_time=float(robot_time),
                    arkit_time_shifted=float(arkit_time_shifted[arkit_index]),
                    T_base_gripper=T_base_gripper,
                    T_world_cam=T_world_cam,
                    T_target_cam=invert_transform(T_world_cam),
                )
            )

        if len(pairs) < 3:
            raise ValueError(
                f"Only {len(pairs)} matched pose pairs found within {self.max_pair_delta:.3f}s. "
                "Increase overlap or relax --max-pair-delta."
            )
        return pairs


class HandEyeRefiner:
    def __init__(self, max_translation_adjustment=0.2, max_rotation_adjustment=0.3):
        self.max_translation_adjustment = max_translation_adjustment
        self.max_rotation_adjustment = max_rotation_adjustment

    def refine(self, hand_eye_result, arkit_series, scale_factor):
        if least_squares is None:
            return hand_eye_result, scale_factor

        pairs = hand_eye_result.matched_pairs
        raw_positions = [arkit_series.position[pair.arkit_index] for pair in pairs]
        world_rotations = [quaternion_to_matrix(arkit_series.quaternion[pair.arkit_index]) for pair in pairs]
        robot_transforms = [pair.T_base_gripper for pair in pairs]

        initial = np.concatenate(
            [
                rotation_matrix_to_rotvec(hand_eye_result.T_base_world[:3, :3]),
                hand_eye_result.T_base_world[:3, 3],
            ]
        )

        lower = np.concatenate(
            [
                initial[0:3] - self.max_rotation_adjustment,
                initial[3:6] - self.max_translation_adjustment,
            ]
        )
        upper = np.concatenate(
            [
                initial[0:3] + self.max_rotation_adjustment,
                initial[3:6] + self.max_translation_adjustment,
            ]
        )

        def residuals(parameters):
            T_base_world = compose_transform(
                rotvec_to_rotation_matrix(parameters[0:3]),
                parameters[3:6],
            )

            residual_vector = []
            for raw_position, world_rotation, T_base_gripper in zip(raw_positions, world_rotations, robot_transforms):
                T_world_cam = compose_transform(world_rotation, raw_position * scale_factor)
                T_base_cam = T_base_world @ T_world_cam
                T_base_gripper_pred = T_base_cam @ invert_transform(hand_eye_result.T_cam2gripper)
                residual_vector.extend((T_base_gripper_pred[:3, 3] - T_base_gripper[:3, 3]) * 1000.0)
            return np.array(residual_vector, dtype=float)

        optimized = least_squares(residuals, initial, bounds=(lower, upper), max_nfev=200)
        parameters = optimized.x
        T_base_world = compose_transform(
            rotvec_to_rotation_matrix(parameters[0:3]),
            parameters[3:6],
        )

        refined_result = HandEyeResult(
            R_cam2gripper=hand_eye_result.R_cam2gripper.copy(),
            t_cam2gripper=hand_eye_result.t_cam2gripper.copy(),
            T_cam2gripper=hand_eye_result.T_cam2gripper.copy(),
            T_base_world=T_base_world,
            matched_pairs=hand_eye_result.matched_pairs,
        )
        return refined_result, scale_factor


class ScaleRefiner:
    def __init__(self, scale_search_ratio=0.3):
        self.scale_search_ratio = scale_search_ratio

    def refine(self, initial_scale, hand_eye_calibrator, hand_eye_refiner, evaluator, robot_series, arkit_series, arkit_time_shifted):
        if minimize_scalar is None:
            return float(initial_scale)

        lower = max(1e-6, float(initial_scale) * (1.0 - self.scale_search_ratio))
        upper = float(initial_scale) * (1.0 + self.scale_search_ratio)

        def objective(scale_value):
            hand_eye_result = hand_eye_calibrator.calibrate(
                robot_series,
                arkit_series,
                arkit_time_shifted,
                float(scale_value),
            )
            refined_hand_eye_result, _ = hand_eye_refiner.refine(hand_eye_result, arkit_series, float(scale_value))
            evaluation = evaluator.evaluate(refined_hand_eye_result)
            return evaluation.mean_absolute_error_mm

        result = minimize_scalar(
            objective,
            bounds=(lower, upper),
            method="bounded",
            options={"xatol": 1e-3, "maxiter": 40},
        )
        if not result.success:
            return float(initial_scale)
        return float(result.x)


class CrossValidator:
    def __init__(self, fold_count=5):
        self.fold_count = fold_count

    def evaluate_scale_generalization(self, base_pairs, initial_scale, hand_eye_calibrator, hand_eye_refiner, evaluator, robot_series, arkit_series, arkit_time_shifted):
        if minimize_scalar is None or len(base_pairs) < self.fold_count * 10:
            return None

        fold_indices = np.array_split(np.arange(len(base_pairs)), self.fold_count)
        fold_results = []

        for fold_index, validation_indices in enumerate(fold_indices, start=1):
            train_mask = np.ones(len(base_pairs), dtype=bool)
            train_mask[validation_indices] = False
            train_templates = [base_pairs[index] for index in np.flatnonzero(train_mask)]
            validation_templates = [base_pairs[index] for index in validation_indices]

            best_scale = self.optimize_scale(
                train_templates,
                initial_scale,
                hand_eye_calibrator,
                hand_eye_refiner,
                evaluator,
                robot_series,
                arkit_series,
                arkit_time_shifted,
            )

            best_hand_eye = self.fit_hand_eye(
                train_templates,
                best_scale,
                hand_eye_calibrator,
                hand_eye_refiner,
                robot_series,
                arkit_series,
                arkit_time_shifted,
            )
            init_hand_eye = self.fit_hand_eye(
                train_templates,
                initial_scale,
                hand_eye_calibrator,
                hand_eye_refiner,
                robot_series,
                arkit_series,
                arkit_time_shifted,
            )

            train_best_eval = self.evaluate_with_pairs(best_hand_eye, train_templates, best_scale, robot_series, arkit_series)
            val_best_eval = self.evaluate_with_pairs(best_hand_eye, validation_templates, best_scale, robot_series, arkit_series)
            train_init_eval = self.evaluate_with_pairs(init_hand_eye, train_templates, initial_scale, robot_series, arkit_series)
            val_init_eval = self.evaluate_with_pairs(init_hand_eye, validation_templates, initial_scale, robot_series, arkit_series)

            fold_results.append(
                CrossValidationFoldResult(
                    fold_index=fold_index,
                    best_scale=best_scale,
                    train_best_abs_mean_mm=train_best_eval.mean_absolute_error_mm,
                    val_best_abs_mean_mm=val_best_eval.mean_absolute_error_mm,
                    train_best_rel_mean_mm=train_best_eval.mean_relative_error_mm,
                    val_best_rel_mean_mm=val_best_eval.mean_relative_error_mm,
                    train_init_abs_mean_mm=train_init_eval.mean_absolute_error_mm,
                    val_init_abs_mean_mm=val_init_eval.mean_absolute_error_mm,
                    train_init_rel_mean_mm=train_init_eval.mean_relative_error_mm,
                    val_init_rel_mean_mm=val_init_eval.mean_relative_error_mm,
                    best_abs_gap_mm=val_best_eval.mean_absolute_error_mm - train_best_eval.mean_absolute_error_mm,
                    init_abs_gap_mm=val_init_eval.mean_absolute_error_mm - train_init_eval.mean_absolute_error_mm,
                )
            )

        best_scales = np.array([fold.best_scale for fold in fold_results], dtype=float)
        train_best_abs = np.array([fold.train_best_abs_mean_mm for fold in fold_results], dtype=float)
        val_best_abs = np.array([fold.val_best_abs_mean_mm for fold in fold_results], dtype=float)
        train_init_abs = np.array([fold.train_init_abs_mean_mm for fold in fold_results], dtype=float)
        val_init_abs = np.array([fold.val_init_abs_mean_mm for fold in fold_results], dtype=float)

        return CrossValidationSummary(
            fold_count=len(fold_results),
            best_scale_mean=float(np.mean(best_scales)),
            best_scale_std=float(np.std(best_scales)),
            train_best_abs_mean_mm=float(np.mean(train_best_abs)),
            val_best_abs_mean_mm=float(np.mean(val_best_abs)),
            best_abs_gap_mean_mm=float(np.mean(val_best_abs - train_best_abs)),
            train_init_abs_mean_mm=float(np.mean(train_init_abs)),
            val_init_abs_mean_mm=float(np.mean(val_init_abs)),
            init_abs_gap_mean_mm=float(np.mean(val_init_abs - train_init_abs)),
            val_abs_improvement_mm=float(np.mean(val_init_abs - val_best_abs)),
            folds=fold_results,
        )

    def optimize_scale(self, train_templates, initial_scale, hand_eye_calibrator, hand_eye_refiner, evaluator, robot_series, arkit_series, arkit_time_shifted):
        def objective(scale_value):
            hand_eye = self.fit_hand_eye(
                train_templates,
                float(scale_value),
                hand_eye_calibrator,
                hand_eye_refiner,
                robot_series,
                arkit_series,
                arkit_time_shifted,
            )
            evaluation = self.evaluate_with_pairs(hand_eye, train_templates, float(scale_value), robot_series, arkit_series)
            return evaluation.mean_absolute_error_mm

        result = minimize_scalar(
            objective,
            bounds=(initial_scale * 0.7, initial_scale * 1.3),
            method="bounded",
            options={"xatol": 1e-3, "maxiter": 40},
        )
        if not result.success:
            return float(initial_scale)
        return float(result.x)

    def fit_hand_eye(self, templates, scale_factor, hand_eye_calibrator, hand_eye_refiner, robot_series, arkit_series, arkit_time_shifted):
        pairs = self.rebuild_pairs(templates, scale_factor, robot_series, arkit_series)
        hand_eye = self.solve_hand_eye_from_pairs(pairs)
        refined_hand_eye, _ = hand_eye_refiner.refine(hand_eye, arkit_series, scale_factor)
        return refined_hand_eye

    def evaluate_with_pairs(self, hand_eye_result, templates, scale_factor, robot_series, arkit_series):
        pairs = self.rebuild_pairs(templates, scale_factor, robot_series, arkit_series)
        evaluation_hand_eye = HandEyeResult(
            R_cam2gripper=hand_eye_result.R_cam2gripper,
            t_cam2gripper=hand_eye_result.t_cam2gripper,
            T_cam2gripper=hand_eye_result.T_cam2gripper,
            T_base_world=hand_eye_result.T_base_world,
            matched_pairs=pairs,
        )
        return CalibrationEvaluator().evaluate(evaluation_hand_eye)

    def rebuild_pairs(self, templates, scale_factor, robot_series, arkit_series):
        rebuilt_pairs = []
        for pair in templates:
            T_base_gripper = pose_to_matrix(robot_series.position[pair.robot_index], robot_series.quaternion[pair.robot_index])
            T_world_cam = pose_to_matrix(
                arkit_series.position[pair.arkit_index],
                arkit_series.quaternion[pair.arkit_index],
                translation_scale=scale_factor,
            )
            rebuilt_pairs.append(
                MatchedFramePair(
                    robot_index=pair.robot_index,
                    arkit_index=pair.arkit_index,
                    time_delta=pair.time_delta,
                    robot_time=pair.robot_time,
                    arkit_time_shifted=pair.arkit_time_shifted,
                    T_base_gripper=T_base_gripper,
                    T_world_cam=T_world_cam,
                    T_target_cam=invert_transform(T_world_cam),
                )
            )
        return rebuilt_pairs

    def solve_hand_eye_from_pairs(self, pairs):
        R_gripper2base = [pair.T_base_gripper[:3, :3] for pair in pairs]
        t_gripper2base = [pair.T_base_gripper[:3, 3].reshape(3, 1) for pair in pairs]
        R_target2cam = [pair.T_target_cam[:3, :3] for pair in pairs]
        t_target2cam = [pair.T_target_cam[:3, 3].reshape(3, 1) for pair in pairs]
        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
            R_gripper2base=R_gripper2base,
            t_gripper2base=t_gripper2base,
            R_target2cam=R_target2cam,
            t_target2cam=t_target2cam,
            method=cv2.CALIB_HAND_EYE_TSAI,
        )
        R_cam2gripper = np.asarray(R_cam2gripper, dtype=float)
        t_cam2gripper = np.asarray(t_cam2gripper, dtype=float).reshape(3)
        T_cam2gripper = compose_transform(R_cam2gripper, t_cam2gripper)
        T_base_world = average_transform([
            pair.T_base_gripper @ T_cam2gripper @ pair.T_target_cam
            for pair in pairs
        ])
        return HandEyeResult(
            R_cam2gripper=R_cam2gripper,
            t_cam2gripper=t_cam2gripper,
            T_cam2gripper=T_cam2gripper,
            T_base_world=T_base_world,
            matched_pairs=pairs,
        )


class StageEvaluator:
    def evaluate_stages(self, hand_eye_result, arkit_series, scale_factor):
        identity_transform = compose_transform(np.eye(3, dtype=float), np.zeros(3, dtype=float))
        stage_specs = [
            ("raw_identity", 1.0, identity_transform, identity_transform),
            ("time_sync_only", 1.0, identity_transform, average_transform([pair.T_base_gripper @ pair.T_target_cam for pair in hand_eye_result.matched_pairs])),
            ("time_sync_fixed_scale", scale_factor, identity_transform, average_transform([
                pair.T_base_gripper @ invert_transform(pose_to_matrix(arkit_series.position[pair.arkit_index], arkit_series.quaternion[pair.arkit_index], translation_scale=scale_factor))
                for pair in hand_eye_result.matched_pairs
            ])),
            ("initial_handeye", scale_factor, hand_eye_result.T_cam2gripper, average_transform([
                pair.T_base_gripper @ hand_eye_result.T_cam2gripper @ invert_transform(pose_to_matrix(
                    arkit_series.position[pair.arkit_index],
                    arkit_series.quaternion[pair.arkit_index],
                    translation_scale=scale_factor,
                ))
                for pair in hand_eye_result.matched_pairs
            ])),
            ("refined_world", scale_factor, hand_eye_result.T_cam2gripper, hand_eye_result.T_base_world),
        ]

        results = []
        for name, stage_scale, T_cam2gripper, T_base_world in stage_specs:
            absolute_errors = self.evaluate_absolute_errors(
                hand_eye_result.matched_pairs,
                arkit_series,
                stage_scale,
                T_cam2gripper,
                T_base_world,
            )
            results.append(
                StageEvaluationResult(
                    name=name,
                    absolute_errors_mm=absolute_errors,
                    mean_absolute_error_mm=float(np.mean(absolute_errors)),
                    rmse_absolute_error_mm=float(np.sqrt(np.mean(np.square(absolute_errors)))),
                    max_absolute_error_mm=float(np.max(absolute_errors)),
                )
            )
        return results

    def evaluate_absolute_errors(self, pairs, arkit_series, scale_factor, T_cam2gripper, T_base_world):
        absolute_errors = []
        for pair in pairs:
            T_world_cam = pose_to_matrix(
                arkit_series.position[pair.arkit_index],
                arkit_series.quaternion[pair.arkit_index],
                translation_scale=scale_factor,
            )
            T_base_cam = T_base_world @ T_world_cam
            T_base_gripper_pred = T_base_cam @ invert_transform(T_cam2gripper)
            absolute_errors.append(
                float(np.linalg.norm((T_base_gripper_pred[:3, 3] - pair.T_base_gripper[:3, 3]) * 1000.0))
            )
        return np.array(absolute_errors, dtype=float)


class CalibrationEvaluator:
    def evaluate(self, hand_eye_result):
        X = hand_eye_result.T_cam2gripper
        pairs = hand_eye_result.matched_pairs
        relative_errors_mm = []
        for start_index in range(0, len(pairs) - 5, 5):
            pair_a = pairs[start_index]
            pair_b = pairs[start_index + 5]
            A = relative_transform(pair_a.T_base_gripper, pair_b.T_base_gripper)
            # OpenCV's eye-in-hand convention solves A X = X B with:
            # A = inv(^bT_g(2)) @ ^bT_g(1)
            # B = ^cT_t(2) @ inv(^cT_t(1))
            # Here our pair ordering is (1 = pair_a, 2 = pair_b).
            B = pair_a.T_target_cam @ invert_transform(pair_b.T_target_cam)
            ax = A @ X
            xb = X @ B
            error_m = float(np.linalg.norm(ax[:3, 3] - xb[:3, 3]))
            relative_errors_mm.append(error_m * 1000.0)

        relative_errors_mm = np.array(relative_errors_mm, dtype=float)
        if relative_errors_mm.size == 0:
            raise ValueError("Not enough matched pose pairs to compute 5-frame relative AX=XB errors.")

        predicted_camera_positions = []
        predicted_gripper_positions = []
        robot_positions = []
        pair_time_deltas_ms = []
        absolute_errors_mm = []
        axis_errors_mm = []
        for pair in pairs:
            T_base_cam = hand_eye_result.T_base_world @ pair.T_world_cam
            T_base_gripper_pred = T_base_cam @ invert_transform(hand_eye_result.T_cam2gripper)
            predicted_camera_positions.append(T_base_cam[:3, 3])
            predicted_gripper_positions.append(T_base_gripper_pred[:3, 3])
            robot_positions.append(pair.T_base_gripper[:3, 3])
            pair_time_deltas_ms.append(pair.time_delta * 1000.0)
            axis_error_mm = (T_base_gripper_pred[:3, 3] - pair.T_base_gripper[:3, 3]) * 1000.0
            axis_errors_mm.append(axis_error_mm)
            absolute_errors_mm.append(
                float(np.linalg.norm(axis_error_mm))
            )

        absolute_errors_mm = np.array(absolute_errors_mm, dtype=float)
        axis_errors_mm = np.array(axis_errors_mm, dtype=float)

        return EvaluationResult(
            relative_errors_mm=relative_errors_mm,
            mean_relative_error_mm=float(np.mean(relative_errors_mm)),
            max_relative_error_mm=float(np.max(relative_errors_mm)),
            absolute_errors_mm=absolute_errors_mm,
            axis_errors_mm=axis_errors_mm,
            mean_absolute_error_mm=float(np.mean(absolute_errors_mm)),
            rmse_absolute_error_mm=float(np.sqrt(np.mean(np.square(absolute_errors_mm)))),
            max_absolute_error_mm=float(np.max(absolute_errors_mm)),
            predicted_camera_positions=np.array(predicted_camera_positions, dtype=float),
            predicted_gripper_positions=np.array(predicted_gripper_positions, dtype=float),
            robot_positions=np.array(robot_positions, dtype=float),
            pair_time_deltas_ms=np.array(pair_time_deltas_ms, dtype=float),
        )


class CalibrationPlotter:
    def __init__(self, output_dir):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

    def create_all(self, result):
        if plt is None:
            raise ImportError("matplotlib is required for plot generation.")
        self.plot_velocity_profiles(result)
        self.plot_staged_absolute_error_curves(result)
        self.plot_cross_validation_summary(result)
        self.plot_relative_error_histogram(result)
        self.plot_absolute_error_histogram(result)
        self.plot_axis_error_curves(result)
        self.plot_absolute_error_comparison(result)
        self.plot_axis_error_comparison(result)
        self.plot_trajectory_overlay(result)

    def plot_velocity_profiles(self, result):
        figure, axis = plt.subplots(figsize=(12, 5))
        axis.plot(
            result.time_sync.robot_profile.time,
            result.time_sync.robot_profile.speed,
            label="Robot speed",
            linewidth=2.0,
            color="#c66a1c",
        )
        axis.plot(
            result.time_sync.arkit_profile.time,
            result.time_sync.arkit_profile.speed,
            label="ARKit speed (raw)",
            linewidth=1.6,
            linestyle="--",
            color="#3d8bfd",
            alpha=0.7,
        )
        axis.plot(
            result.scale.arkit_scaled_profile.time,
            result.scale.arkit_scaled_profile.speed,
            label="ARKit speed (time-aligned, scale=1)",
            linewidth=2.0,
            color="#0d9488",
        )
        axis.set_title("Velocity-Time Curves Before And After Alignment")
        axis.set_xlabel("Time (s)")
        axis.set_ylabel("Speed (m/s)")
        axis.grid(True, alpha=0.25)
        axis.legend()
        figure.tight_layout()
        figure.savefig(self.output_dir / "velocity_time_alignment.png", dpi=180)
        plt.close(figure)

    def plot_staged_absolute_error_curves(self, result):
        figure, axis = plt.subplots(figsize=(12, 6))
        time_axis = np.array([pair.robot_time for pair in result.hand_eye.matched_pairs], dtype=float)
        color_map = {
            "raw_identity": "#94a3b8",
            "time_sync_only": "#f59e0b",
            "time_sync_fixed_scale": "#8b5cf6",
            "initial_handeye": "#2563eb",
            "refined_world": "#059669",
        }
        label_map = {
            "raw_identity": "1. Raw identity",
            "time_sync_only": "2. Time sync only",
            "time_sync_fixed_scale": "3. Time sync + fixed scale",
            "initial_handeye": "4. Initial hand-eye",
            "refined_world": "5. Optional world refinement",
        }
        for stage in result.stage_evaluations:
            axis.plot(
                time_axis,
                stage.absolute_errors_mm,
                label=label_map.get(stage.name, stage.name),
                linewidth=1.9,
                color=color_map.get(stage.name),
            )
        axis.set_title("Absolute Error Across Calibration Stages")
        axis.set_xlabel("Robot time (s)")
        axis.set_ylabel("Absolute position error (mm)")
        axis.grid(True, alpha=0.25)
        axis.legend()
        figure.tight_layout()
        figure.savefig(self.output_dir / "absolute_error_stages.png", dpi=180)
        plt.close(figure)

    def plot_cross_validation_summary(self, result):
        if result.cross_validation is None:
            return
        figure, axis = plt.subplots(figsize=(10, 5))
        folds = [fold.fold_index for fold in result.cross_validation.folds]
        train_values = [fold.train_best_abs_mean_mm for fold in result.cross_validation.folds]
        val_values = [fold.val_best_abs_mean_mm for fold in result.cross_validation.folds]
        axis.plot(folds, train_values, marker="o", linewidth=1.8, label="Train mean abs error", color="#2563eb")
        axis.plot(folds, val_values, marker="o", linewidth=1.8, label="Validation mean abs error", color="#dc2626")
        axis.set_title("Scale Cross-Validation Summary")
        axis.set_xlabel("Fold")
        axis.set_ylabel("Mean absolute error (mm)")
        axis.set_xticks(folds)
        axis.grid(True, alpha=0.25)
        axis.legend()
        figure.tight_layout()
        figure.savefig(self.output_dir / "scale_cross_validation.png", dpi=180)
        plt.close(figure)

    def plot_relative_error_histogram(self, result):
        figure, axis = plt.subplots(figsize=(8, 5))
        axis.hist(result.evaluation.relative_errors_mm, bins=min(30, len(result.evaluation.relative_errors_mm)), color="#2563eb", alpha=0.85)
        axis.set_title("Relative Translation Error Histogram")
        axis.set_xlabel("Relative error (mm)")
        axis.set_ylabel("Count")
        axis.grid(True, alpha=0.25)
        figure.tight_layout()
        figure.savefig(self.output_dir / "relative_error_histogram.png", dpi=180)
        plt.close(figure)

    def plot_absolute_error_histogram(self, result):
        figure, axis = plt.subplots(figsize=(8, 5))
        axis.hist(result.evaluation.absolute_errors_mm, bins=min(30, len(result.evaluation.absolute_errors_mm)), color="#0891b2", alpha=0.85)
        axis.set_title("Absolute End-Effector Error Histogram")
        axis.set_xlabel("Absolute position error (mm)")
        axis.set_ylabel("Count")
        axis.grid(True, alpha=0.25)
        figure.tight_layout()
        figure.savefig(self.output_dir / "absolute_error_histogram.png", dpi=180)
        plt.close(figure)

    def plot_axis_error_curves(self, result):
        figure, axis = plt.subplots(figsize=(12, 5))
        time_axis = np.array([pair.robot_time for pair in result.hand_eye.matched_pairs], dtype=float)
        axis_errors = result.evaluation.axis_errors_mm
        axis.plot(time_axis, axis_errors[:, 0], label="X error", linewidth=1.8, color="#dc2626")
        axis.plot(time_axis, axis_errors[:, 1], label="Y error", linewidth=1.8, color="#2563eb")
        axis.plot(time_axis, axis_errors[:, 2], label="Z error", linewidth=1.8, color="#059669")
        axis.axhline(0.0, color="#444", linewidth=1.0, alpha=0.5)
        axis.set_title("Per-Axis End-Effector Error Curves")
        axis.set_xlabel("Robot time (s)")
        axis.set_ylabel("Axis error (mm)")
        axis.grid(True, alpha=0.25)
        axis.legend()
        figure.tight_layout()
        figure.savefig(self.output_dir / "axis_error_curves.png", dpi=180)
        plt.close(figure)

    def plot_absolute_error_comparison(self, result):
        figure, axis = plt.subplots(figsize=(12, 5))
        time_axis = np.array([pair.robot_time for pair in result.hand_eye.matched_pairs], dtype=float)
        axis.plot(
            time_axis,
            result.initial_evaluation.absolute_errors_mm,
            label="Before refinement",
            linewidth=1.8,
            color="#94a3b8",
        )
        axis.plot(
            time_axis,
            result.evaluation.absolute_errors_mm,
            label="After refinement",
            linewidth=2.0,
            color="#0f766e",
        )
        axis.set_title("Absolute Error Before Vs After Refinement")
        axis.set_xlabel("Robot time (s)")
        axis.set_ylabel("Absolute position error (mm)")
        axis.grid(True, alpha=0.25)
        axis.legend()
        figure.tight_layout()
        figure.savefig(self.output_dir / "absolute_error_comparison.png", dpi=180)
        plt.close(figure)

    def plot_axis_error_comparison(self, result):
        figure, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
        time_axis = np.array([pair.robot_time for pair in result.hand_eye.matched_pairs], dtype=float)
        before = result.initial_evaluation.axis_errors_mm
        after = result.evaluation.axis_errors_mm
        labels = ["X", "Y", "Z"]
        colors = ["#dc2626", "#2563eb", "#059669"]

        for axis_index, subplot in enumerate(axes):
            subplot.plot(time_axis, before[:, axis_index], label=f"{labels[axis_index]} before", linewidth=1.5, color="#cbd5e1")
            subplot.plot(time_axis, after[:, axis_index], label=f"{labels[axis_index]} after", linewidth=1.8, color=colors[axis_index])
            subplot.axhline(0.0, color="#444", linewidth=1.0, alpha=0.4)
            subplot.set_ylabel(f"{labels[axis_index]} (mm)")
            subplot.grid(True, alpha=0.25)
            subplot.legend(loc="upper right")

        axes[-1].set_xlabel("Robot time (s)")
        figure.suptitle("Per-Axis Error Before Vs After Refinement")
        figure.tight_layout()
        figure.savefig(self.output_dir / "axis_error_comparison.png", dpi=180)
        plt.close(figure)

    def plot_trajectory_overlay(self, result):
        figure = plt.figure(figsize=(8, 7))
        axis = figure.add_subplot(111, projection="3d")
        robot = result.evaluation.robot_positions
        predicted = result.evaluation.predicted_gripper_positions
        axis.plot(robot[:, 0], robot[:, 1], robot[:, 2], label="Robot gripper (measured)", linewidth=2.2, color="#d97706")
        axis.plot(predicted[:, 0], predicted[:, 1], predicted[:, 2], label="ARKit gripper (calibrated)", linewidth=2.0, color="#0284c7")
        axis.set_title("Final 3D End-Effector Trajectory Overlap")
        axis.set_xlabel("X (m)")
        axis.set_ylabel("Y (m)")
        axis.set_zlabel("Z (m)")
        axis.legend()
        self._set_equal_axes(axis, np.vstack([robot, predicted]))
        figure.tight_layout()
        figure.savefig(self.output_dir / "trajectory_overlap_3d.png", dpi=180)
        plt.close(figure)

    @staticmethod
    def _set_equal_axes(axis, points):
        minimum = points.min(axis=0)
        maximum = points.max(axis=0)
        center = (minimum + maximum) * 0.5
        radius = float(np.max(maximum - minimum)) * 0.5
        radius = max(radius, 1e-6)
        axis.set_xlim(center[0] - radius, center[0] + radius)
        axis.set_ylim(center[1] - radius, center[1] + radius)
        axis.set_zlim(center[2] - radius, center[2] + radius)


class OfflinePoseCalibrationPipeline:
    def __init__(self, max_pair_delta=0.05, smooth_window=11, cross_validate_scale=False, cv_folds=5, skip_world_refinement=False):
        self.loader = PoseDataLoader()
        self.time_sync = TimeSynchronizer(smooth_window=smooth_window)
        self.scale_calibrator = ScaleCalibrator()
        self.hand_eye_calibrator = HandEyeCalibrator(max_pair_delta=max_pair_delta)
        self.hand_eye_refiner = HandEyeRefiner()
        self.evaluator = CalibrationEvaluator()
        self.stage_evaluator = StageEvaluator()
        self.cross_validate_scale = False
        self.skip_world_refinement = skip_world_refinement

    def run(self, arkit_csv, robot_csv):
        arkit_series = self.loader.load_series(arkit_csv, "arkit")
        robot_series = self.loader.load_series(robot_csv, "robot")
        time_sync_result = self.time_sync.synchronize(robot_series, arkit_series)
        scale_result = self.scale_calibrator.calibrate(
            time_sync_result.robot_profile,
            time_sync_result.arkit_profile,
            time_sync_result.arkit_time_shifted,
        )
        hand_eye_result = self.hand_eye_calibrator.calibrate(
            robot_series,
            arkit_series,
            time_sync_result.arkit_time_shifted,
            1.0,
        )
        initial_evaluation = self.evaluator.evaluate(hand_eye_result)
        if self.skip_world_refinement:
            refined_hand_eye_result = hand_eye_result
        else:
            refined_hand_eye_result, _ = self.hand_eye_refiner.refine(
                hand_eye_result,
                arkit_series,
                1.0,
            )
        evaluation_result = self.evaluator.evaluate(refined_hand_eye_result)
        stage_evaluations = self.stage_evaluator.evaluate_stages(
            refined_hand_eye_result,
            arkit_series,
            1.0,
        )
        cross_validation = None
        return OfflineCalibrationResult(
            time_sync=time_sync_result,
            scale=scale_result,
            initial_hand_eye=hand_eye_result,
            initial_evaluation=initial_evaluation,
            hand_eye=refined_hand_eye_result,
            evaluation=evaluation_result,
            stage_evaluations=stage_evaluations,
            cross_validation=cross_validation,
        )


def transform_realtime_arkit_pose(position, quaternion, scale_factor, R_cam2gripper, t_cam2gripper, T_base_world):
    """
    Convert a live ARKit camera pose into the robot base frame using the offline validation result.

    The input pose is ARKit's raw world->camera pose expressed as position + quaternion.
    The output is a 4x4 camera pose in the robot base frame. If you need the live gripper pose,
    right-multiply by the inverse of T_cam2gripper.
    """
    T_world_cam = pose_to_matrix(position, quaternion, translation_scale=scale_factor)
    T_cam2gripper = compose_transform(R_cam2gripper, t_cam2gripper)
    return T_base_world @ T_world_cam, T_base_world @ T_world_cam @ invert_transform(T_cam2gripper)


def serialize_offline_result(result):
    refinement_gain_mm = result.initial_evaluation.mean_absolute_error_mm - result.evaluation.mean_absolute_error_mm
    payload = {
        "time_shift": result.time_shift,
        "scale_factor": result.scale_factor,
        "initial_scale_factor": result.scale.initial_scale_factor,
        "max_v_robot": result.scale.max_v_robot,
        "max_v_arkit": result.scale.max_v_arkit,
        "R_cam2gripper": result.hand_eye.R_cam2gripper.tolist(),
        "t_cam2gripper": result.hand_eye.t_cam2gripper.tolist(),
        "T_cam2gripper": result.hand_eye.T_cam2gripper.tolist(),
        "T_base_world": result.hand_eye.T_base_world.tolist(),
        "matched_pair_count": len(result.hand_eye.matched_pairs),
        "initial_mean_relative_error_mm": result.initial_evaluation.mean_relative_error_mm,
        "initial_max_relative_error_mm": result.initial_evaluation.max_relative_error_mm,
        "initial_mean_absolute_error_mm": result.initial_evaluation.mean_absolute_error_mm,
        "initial_rmse_absolute_error_mm": result.initial_evaluation.rmse_absolute_error_mm,
        "initial_max_absolute_error_mm": result.initial_evaluation.max_absolute_error_mm,
        "refinement_gain_mm": refinement_gain_mm,
        "mean_relative_error_mm": result.evaluation.mean_relative_error_mm,
        "max_relative_error_mm": result.evaluation.max_relative_error_mm,
        "mean_absolute_error_mm": result.evaluation.mean_absolute_error_mm,
        "rmse_absolute_error_mm": result.evaluation.rmse_absolute_error_mm,
        "max_absolute_error_mm": result.evaluation.max_absolute_error_mm,
        "stage_absolute_errors": [
            {
                "name": stage.name,
                "mean_absolute_error_mm": stage.mean_absolute_error_mm,
                "rmse_absolute_error_mm": stage.rmse_absolute_error_mm,
                "max_absolute_error_mm": stage.max_absolute_error_mm,
            }
            for stage in result.stage_evaluations
        ],
        "mean_pair_time_delta_ms": float(np.mean(result.evaluation.pair_time_deltas_ms)),
        "max_pair_time_delta_ms": float(np.max(result.evaluation.pair_time_deltas_ms)),
    }
    if result.cross_validation is not None:
        payload["cross_validation"] = {
            "fold_count": result.cross_validation.fold_count,
            "best_scale_mean": result.cross_validation.best_scale_mean,
            "best_scale_std": result.cross_validation.best_scale_std,
            "train_best_abs_mean_mm": result.cross_validation.train_best_abs_mean_mm,
            "val_best_abs_mean_mm": result.cross_validation.val_best_abs_mean_mm,
            "best_abs_gap_mean_mm": result.cross_validation.best_abs_gap_mean_mm,
            "train_init_abs_mean_mm": result.cross_validation.train_init_abs_mean_mm,
            "val_init_abs_mean_mm": result.cross_validation.val_init_abs_mean_mm,
            "init_abs_gap_mean_mm": result.cross_validation.init_abs_gap_mean_mm,
            "val_abs_improvement_mm": result.cross_validation.val_abs_improvement_mm,
            "folds": [
                {
                    "fold_index": fold.fold_index,
                    "best_scale": fold.best_scale,
                    "train_best_abs_mean_mm": fold.train_best_abs_mean_mm,
                    "val_best_abs_mean_mm": fold.val_best_abs_mean_mm,
                    "train_best_rel_mean_mm": fold.train_best_rel_mean_mm,
                    "val_best_rel_mean_mm": fold.val_best_rel_mean_mm,
                    "train_init_abs_mean_mm": fold.train_init_abs_mean_mm,
                    "val_init_abs_mean_mm": fold.val_init_abs_mean_mm,
                    "train_init_rel_mean_mm": fold.train_init_rel_mean_mm,
                    "val_init_rel_mean_mm": fold.val_init_rel_mean_mm,
                    "best_abs_gap_mm": fold.best_abs_gap_mm,
                    "init_abs_gap_mm": fold.init_abs_gap_mm,
                }
                for fold in result.cross_validation.folds
            ],
        }
    return payload


def print_offline_result(result):
    refinement_gain_mm = result.initial_evaluation.mean_absolute_error_mm - result.evaluation.mean_absolute_error_mm
    refinement_label = "useful" if refinement_gain_mm > 0.2 else "negligible"
    print("=== Offline Validation Result ===")
    print(f"time_shift: {result.time_shift:.6f} s")
    print(f"scale_factor: {result.scale_factor:.6f} (fixed)")
    print(f"matched_pose_pairs: {len(result.hand_eye.matched_pairs)}")
    print(f"initial_relative_error_mean: {result.initial_evaluation.mean_relative_error_mm:.3f} mm")
    print(f"initial_absolute_error_mean: {result.initial_evaluation.mean_absolute_error_mm:.3f} mm")
    print(f"relative_error_mean: {result.evaluation.mean_relative_error_mm:.3f} mm")
    print(f"relative_error_max: {result.evaluation.max_relative_error_mm:.3f} mm")
    print(f"absolute_error_mean: {result.evaluation.mean_absolute_error_mm:.3f} mm")
    print(f"absolute_error_rmse: {result.evaluation.rmse_absolute_error_mm:.3f} mm")
    print(f"absolute_error_max: {result.evaluation.max_absolute_error_mm:.3f} mm")
    print(f"refinement_gain_mm: {refinement_gain_mm:.3f} ({refinement_label})")
    print(f"pair_time_delta_mean: {float(np.mean(result.evaluation.pair_time_deltas_ms)):.3f} ms")
    print(f"pair_time_delta_max: {float(np.max(result.evaluation.pair_time_deltas_ms)):.3f} ms")
    if result.cross_validation is not None:
        cv = result.cross_validation
        print(
            "cross_validation: "
            f"best_scale {cv.best_scale_mean:.6f} +/- {cv.best_scale_std:.6f} | "
            f"train abs {cv.train_best_abs_mean_mm:.3f} mm | "
            f"val abs {cv.val_best_abs_mean_mm:.3f} mm | "
            f"gap {cv.best_abs_gap_mean_mm:.3f} mm | "
            f"val improvement {cv.val_abs_improvement_mm:.3f} mm"
        )
    print("stage_absolute_error_summary:")
    for stage in result.stage_evaluations:
        print(
            f"  {stage.name}: mean {stage.mean_absolute_error_mm:.3f} mm | "
            f"rmse {stage.rmse_absolute_error_mm:.3f} mm | max {stage.max_absolute_error_mm:.3f} mm"
        )
    print("R_cam2gripper:")
    print(np.array2string(result.hand_eye.R_cam2gripper, precision=6, suppress_small=True))
    print("t_cam2gripper:")
    print(np.array2string(result.hand_eye.t_cam2gripper, precision=6, suppress_small=True))
    print("T_base_world:")
    print(np.array2string(result.hand_eye.T_base_world, precision=6, suppress_small=True))


def run_offline_calibration_to_output(
    arkit_csv,
    sensor_csv,
    output_root="offline_calibration_output",
    max_pair_delta=0.05,
    smooth_window=11,
    cross_validate_scale=False,
    cv_folds=5,
    skip_world_refinement=False,
):
    pipeline = OfflinePoseCalibrationPipeline(
        max_pair_delta=max_pair_delta,
        smooth_window=smooth_window,
        cross_validate_scale=cross_validate_scale,
        cv_folds=cv_folds,
        skip_world_refinement=skip_world_refinement,
    )
    result = pipeline.run(arkit_csv, sensor_csv)
    output_dir = prepare_run_output_dir(output_root, arkit_csv, sensor_csv)
    CalibrationPlotter(output_dir).create_all(result)
    result_path = output_dir / "offline_calibration_result.json"
    result_path.write_text(json.dumps(serialize_offline_result(result), indent=2), encoding="utf-8")
    return result, output_dir, result_path


class OfflineCalibrationWorker(QThread):
    status_changed = pyqtSignal(str)
    finished_successfully = pyqtSignal(object, str, str)
    failed = pyqtSignal(str)

    def __init__(
        self,
        arkit_csv,
        sensor_csv,
        output_root,
        max_pair_delta,
        smooth_window,
        cross_validate_scale,
        cv_folds,
        skip_world_refinement,
    ):
        super().__init__()
        self.arkit_csv = arkit_csv
        self.sensor_csv = sensor_csv
        self.output_root = output_root
        self.max_pair_delta = max_pair_delta
        self.smooth_window = smooth_window
        self.cross_validate_scale = cross_validate_scale
        self.cv_folds = cv_folds
        self.skip_world_refinement = skip_world_refinement

    def run(self):
        try:
            self.status_changed.emit("Running offline validation...")
            result, output_dir, result_path = run_offline_calibration_to_output(
                self.arkit_csv,
                self.sensor_csv,
                output_root=self.output_root,
                max_pair_delta=self.max_pair_delta,
                smooth_window=self.smooth_window,
                cross_validate_scale=self.cross_validate_scale,
                cv_folds=self.cv_folds,
                skip_world_refinement=self.skip_world_refinement,
            )
            self.finished_successfully.emit(result, str(output_dir), str(result_path))
        except Exception as exc:
            self.failed.emit(str(exc))


class AdaptiveCalibrator:
    def __init__(self, max_time_offset=0.5, offset_step=0.02, pairing_window=0.04, min_pairs=20, min_motion_span=0.05):
        self.max_time_offset = max_time_offset
        self.offset_step = offset_step
        self.pairing_window = pairing_window
        self.min_pairs = min_pairs
        self.min_motion_span = min_motion_span
        self.result = CalibrationResult()

    def update(self, arkit_samples, sensor_samples):
        if len(arkit_samples) < self.min_pairs or len(sensor_samples) < self.min_pairs:
            self.result = CalibrationResult()
            return self.result

        arkit_list = list(arkit_samples)
        sensor_list = list(sensor_samples)
        candidate_offsets = np.arange(-self.max_time_offset, self.max_time_offset + self.offset_step * 0.5, self.offset_step)
        best = None

        for offset in candidate_offsets:
            pairs = self.make_pairs(arkit_list, sensor_list, time_offset=offset, time_slope=1.0)
            if len(pairs) < self.min_pairs:
                continue
            if not self.has_enough_motion(pairs):
                continue
            transform, position_rmse = self.estimate_similarity(pairs)
            score = self.normalized_position_score(pairs, transform, position_rmse)
            if best is None or score < best[0]:
                best = (score, offset, pairs, transform, position_rmse)

        if best is None:
            self.result = CalibrationResult()
            return self.result

        _, offset, pairs, transform, position_rmse = best
        time_slope, time_offset = self.estimate_time_model(pairs)
        pairs = self.make_pairs(arkit_list, sensor_list, time_offset=time_offset, time_slope=time_slope)
        if len(pairs) >= self.min_pairs:
            transform, position_rmse = self.estimate_similarity(pairs)
            pairs, inlier_ratio = self.filter_inliers(pairs, transform)
            if len(pairs) >= self.min_pairs:
                transform, position_rmse = self.estimate_similarity(pairs)
        else:
            time_slope = 1.0
            time_offset = offset
            inlier_ratio = 1.0

        angle_rmse = self.estimate_orientation_delta(transform, pairs)

        result = CalibrationResult()
        result.enabled = True
        result.transform = transform
        result.time_offset = float(time_offset)
        result.time_slope = float(time_slope)
        result.mean_time_error, result.time_rmse, result.max_time_error = self.time_error_stats(pairs, time_offset, time_slope)
        result.position_rmse = position_rmse
        result.angle_rmse = angle_rmse
        result.pair_count = len(pairs)
        result.scale = transform.scale
        result.motion_coverage = self.motion_coverage(pairs)
        result.inlier_ratio = inlier_ratio
        result.quality = self.quality_label(result)
        self.result = result
        return result

    def sample_time(self, sample):
        return sample.sensor_time if sample.sensor_time is not None else sample.sender_time

    def make_pairs(self, arkit_samples, sensor_samples, time_offset, time_slope=1.0):
        sensor_times = [time_slope * self.sample_time(sample) + time_offset for sample in sensor_samples]
        pairs = []
        for arkit in arkit_samples:
            index = bisect.bisect_left(sensor_times, arkit.sender_time)
            candidates = []
            if index < len(sensor_samples):
                candidates.append((abs(sensor_times[index] - arkit.sender_time), sensor_samples[index]))
            if index > 0:
                candidates.append((abs(sensor_times[index - 1] - arkit.sender_time), sensor_samples[index - 1]))
            if not candidates:
                continue
            delta, sensor = min(candidates, key=lambda item: item[0])
            if delta <= self.pairing_window:
                pairs.append((arkit, sensor))
        return pairs

    def estimate_time_model(self, pairs):
        sensor_times = np.array([self.sample_time(pair[1]) for pair in pairs], dtype=float)
        arkit_times = np.array([pair[0].sender_time for pair in pairs], dtype=float)
        if len(sensor_times) < 2 or float(np.ptp(sensor_times)) < 1e-6:
            return 1.0, float(np.mean(arkit_times - sensor_times))

        slope, intercept = np.polyfit(sensor_times, arkit_times, 1)
        slope = float(np.clip(slope, 0.98, 1.02))
        intercept = float(np.mean(arkit_times - slope * sensor_times))
        return slope, intercept

    def filter_inliers(self, pairs, transform):
        errors = np.array([
            np.linalg.norm(transform.apply_position(pair[1].position) - pair[0].position)
            for pair in pairs
        ], dtype=float)
        if len(errors) < self.min_pairs:
            return pairs, 1.0
        median = float(np.median(errors))
        mad = float(np.median(np.abs(errors - median)))
        threshold = median + max(0.03, 3.0 * 1.4826 * mad)
        inliers = [pair for pair, error in zip(pairs, errors) if error <= threshold]
        if len(inliers) < self.min_pairs:
            return pairs, 1.0
        return inliers, len(inliers) / len(pairs)

    def has_enough_motion(self, pairs):
        arkit_points = np.array([pair[0].position for pair in pairs], dtype=float)
        sensor_points = np.array([pair[1].position for pair in pairs], dtype=float)
        if len(arkit_points) < self.min_pairs:
            return False
        arkit_span = float(np.max(np.linalg.norm(arkit_points - arkit_points.mean(axis=0), axis=1)))
        sensor_span = float(np.max(np.linalg.norm(sensor_points - sensor_points.mean(axis=0), axis=1)))
        return arkit_span >= self.min_motion_span and sensor_span >= self.min_motion_span

    def normalized_position_score(self, pairs, transform, position_rmse):
        arkit_points = np.array([pair[0].position for pair in pairs], dtype=float)
        span = float(np.max(np.linalg.norm(arkit_points - arkit_points.mean(axis=0), axis=1)))
        return position_rmse / max(span, 1e-3)

    def estimate_similarity(self, pairs):
        arkit_points = np.array([pair[0].position for pair in pairs], dtype=float)
        sensor_points = np.array([pair[1].position for pair in pairs], dtype=float)
        sensor_mean = sensor_points.mean(axis=0)
        arkit_mean = arkit_points.mean(axis=0)
        sensor_centered = sensor_points - sensor_mean
        arkit_centered = arkit_points - arkit_mean

        covariance = sensor_centered.T @ arkit_centered / len(pairs)
        u, singular_values, vh = np.linalg.svd(covariance)
        correction = np.eye(3, dtype=float)
        if np.linalg.det(vh.T @ u.T) < 0:
            correction[2, 2] = -1.0
        rotation = vh.T @ correction @ u.T
        variance = float(np.mean(np.sum(sensor_centered * sensor_centered, axis=1)))
        scale = 1.0 if variance < 1e-9 else float(np.sum(singular_values * np.diag(correction)) / variance)
        translation = arkit_mean - scale * (rotation @ sensor_mean)

        transform = SimilarityTransform(scale=scale, rotation=rotation, translation=translation)
        aligned = np.array([transform.apply_position(point) for point in sensor_points], dtype=float)
        rmse = float(np.sqrt(np.mean(np.sum((aligned - arkit_points) ** 2, axis=1))))
        return transform, rmse

    def time_error_stats(self, pairs, offset, slope=1.0):
        errors = np.array([slope * self.sample_time(pair[1]) + offset - pair[0].sender_time for pair in pairs], dtype=float)
        mean_error = float(np.mean(errors))
        rmse = float(np.sqrt(np.mean(errors * errors)))
        max_error = float(np.max(np.abs(errors)))
        return mean_error, rmse, max_error

    def motion_coverage(self, pairs):
        arkit_points = np.array([pair[0].position for pair in pairs], dtype=float)
        if len(arkit_points) < 2:
            return "none"
        span = np.ptp(arkit_points, axis=0)
        axes = [name for name, value in zip(["x", "y", "z"], span) if value >= self.min_motion_span]
        return "none" if not axes else "".join(axes)

    def quality_label(self, result):
        if result.pair_count < self.min_pairs:
            return "waiting"
        coverage_count = 0 if result.motion_coverage == "none" else len(result.motion_coverage)
        position_ok = result.position_rmse is not None and result.position_rmse < 0.05
        timing_ok = result.time_rmse is not None and result.time_rmse < 0.02
        if coverage_count >= 3 and position_ok and timing_ok and result.inlier_ratio >= 0.75:
            return "good"
        if coverage_count >= 2 and result.inlier_ratio >= 0.5:
            return "weak"
        return "unstable"

    def estimate_orientation_delta(self, transform, pairs):
        rotation_quaternion = matrix_to_quaternion(transform.rotation)
        deltas = []
        errors = []

        for arkit, sensor in pairs:
            rotated_sensor = quaternion_multiply(rotation_quaternion, sensor.quaternion)
            delta = quaternion_multiply(arkit.quaternion, quaternion_conjugate(rotated_sensor))
            deltas.append(delta)

        transform.orientation_delta = average_quaternion(deltas)

        for arkit, sensor in pairs:
            aligned_sensor = transform.apply_quaternion(sensor.quaternion)
            errors.append(quaternion_angle_error_degrees(arkit.quaternion, aligned_sensor))

        if not errors:
            return None
        return float(np.sqrt(np.mean(np.square(errors))))


class PoseReceiverThread(QThread):
    sample_received = pyqtSignal(object)
    status_updated = pyqtSignal(str, dict)
    error_occurred = pyqtSignal(str, str)

    def __init__(self, stream_name, port, host="0.0.0.0"):
        super().__init__()
        self.stream_name = stream_name
        self.host = host
        self.port = port
        self.running = False
        self.prev_sequence = None
        self.prev_recv_time = None
        self.packet_count = 0
        self.drop_count = 0
        self.start_time = 0.0

    def run(self):
        self.running = True
        self.start_time = time.time()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        try:
            sock.bind((self.host, self.port))
            sock.settimeout(0.1)
            while self.running:
                try:
                    packet, address = sock.recvfrom(4096)
                except socket.timeout:
                    continue

                recv_time = time.time()
                monotonic_time = time.monotonic()

                try:
                    sequence, sender_time, position, quaternion, sensor_time, protocol_version, checksum_valid = decode_packet(packet)
                except Exception as exc:
                    self.error_occurred.emit(self.stream_name, str(exc))
                    continue

                if self.prev_sequence is not None:
                    self.drop_count += max(0, sequence - self.prev_sequence - 1)
                self.prev_sequence = sequence

                fps = 0.0
                if self.prev_recv_time is not None:
                    fps = 1.0 / max(monotonic_time - self.prev_recv_time, 1e-9)
                self.prev_recv_time = monotonic_time
                self.packet_count += 1

                sample = PoseSample(
                    stream=self.stream_name,
                    sequence=sequence,
                    sender_time=sender_time,
                    recv_time=recv_time,
                    position=position,
                    quaternion=quaternion,
                    sensor_time=sensor_time,
                    protocol_version=protocol_version,
                    checksum_valid=checksum_valid,
                )
                self.sample_received.emit(sample)
                self.status_updated.emit(
                    self.stream_name,
                    {
                        "address": f"{address[0]}:{address[1]}",
                        "fps": fps,
                        "packets": self.packet_count,
                        "drops": self.drop_count,
                        "latency_ms": max(0.0, (recv_time - sender_time) * 1000.0),
                        "protocol_version": protocol_version,
                    },
                )
        except Exception as exc:
            self.error_occurred.emit(self.stream_name, f"Socket error: {exc}")
        finally:
            sock.close()

    def stop(self):
        self.running = False


class PoseTrack:
    def __init__(self, max_samples=12000):
        self.samples = deque(maxlen=max_samples)
        self.origin = None

    def append(self, sample):
        if self.origin is None:
            self.origin = sample.position.copy()
        self.samples.append(sample)

    def reset_origin(self):
        if self.samples:
            self.origin = self.samples[-1].position.copy()

    def positions(self, last_seconds=None):
        if not self.samples:
            return np.empty((0, 3), dtype=float)

        # For CSV data, recv_time is relative time, not Unix timestamp
        # Only apply time filtering for live data (recv_time > 1e9 indicates Unix timestamp)
        cutoff = None
        if last_seconds is not None and self.samples and self.samples[0].recv_time > 1e9:
            cutoff = time.time() - last_seconds

        points = []
        origin = self.origin if self.origin is not None else np.zeros(3, dtype=float)
        for sample in self.samples:
            if cutoff is not None and sample.recv_time < cutoff:
                continue
            points.append(sample.position - origin)
        if not points:
            return np.empty((0, 3), dtype=float)
        return np.array(points, dtype=float)

    def latest_relative(self):
        if not self.samples:
            return None
        origin = self.origin if self.origin is not None else np.zeros(3, dtype=float)
        sample = self.samples[-1]
        return PoseSample(
            stream=sample.stream,
            sequence=sample.sequence,
            sender_time=sample.sender_time,
            recv_time=sample.recv_time,
            position=sample.position - origin,
            quaternion=sample.quaternion,
            sensor_time=sample.sensor_time,
            protocol_version=sample.protocol_version,
            checksum_valid=sample.checksum_valid,
        )


class CalibratedSensorTrack:
    def __init__(self, source_track, calibration_result):
        self.source_track = source_track
        self.calibration_result = calibration_result
        self.origin = None

    def positions(self, last_seconds=None, reference_origin=None):
        if not self.calibration_result.enabled:
            print(f"CalibratedSensorTrack.positions: calibration not enabled!")
            return np.empty((0, 3), dtype=float)

        if not self.source_track.samples:
            print(f"CalibratedSensorTrack.positions: no source samples!")
            return np.empty((0, 3), dtype=float)

        # For CSV data, recv_time is relative time, not Unix timestamp
        # Only apply time filtering for live data (recv_time > 1e9 indicates Unix timestamp)
        cutoff = None
        if last_seconds is not None and self.source_track.samples and self.source_track.samples[0].recv_time > 1e9:
            cutoff = time.time() - last_seconds

        points = []
        for sample in self.source_track.samples:
            if cutoff is not None and sample.recv_time < cutoff:
                continue
            points.append(self.calibration_result.transform.apply_position(sample.position))

        if not points:
            print(f"CalibratedSensorTrack.positions: no points after filtering!")
            return np.empty((0, 3), dtype=float)

        points = np.array(points, dtype=float)
        print(f"CalibratedSensorTrack.positions: returning {len(points)} points")
        if reference_origin is not None:
            return points - reference_origin
        if self.origin is None:
            self.origin = points[0].copy()
        return points - self.origin

    def latest_relative(self, reference_origin=None):
        if not self.calibration_result.enabled or not self.source_track.samples:
            return None

        sample = self.source_track.samples[-1]
        position = self.calibration_result.transform.apply_position(sample.position)
        if reference_origin is None and self.origin is None:
            self.origin = position.copy()
        origin = reference_origin if reference_origin is not None else self.origin

        return PoseSample(
            stream=sample.stream,
            sequence=sample.sequence,
            sender_time=sample.sender_time,
            recv_time=sample.recv_time,
            position=position - origin,
            quaternion=self.calibration_result.transform.apply_quaternion(sample.quaternion),
            sensor_time=sample.sensor_time,
            protocol_version=sample.protocol_version,
            checksum_valid=sample.checksum_valid,
        )

    def reset_origin(self):
        self.origin = None
        if self.calibration_result.enabled and self.source_track.samples:
            sample = self.source_track.samples[-1]
            self.origin = self.calibration_result.transform.apply_position(sample.position)


class ExternalCameraView(gl.GLViewWidget):
    def __init__(self):
        super().__init__()
        self.setBackgroundColor("#101418")
        self.setCameraPosition(distance=2.5, elevation=28, azimuth=42)
        self.last_camera_fit = 0.0

        grid = gl.GLGridItem()
        grid.setSize(4.0, 4.0, 1.0)
        grid.setSpacing(0.25, 0.25, 0.25)
        grid.setColor((60, 70, 80, 90))
        self.addItem(grid)

        self.add_axes()

        self.arkit_line = gl.GLLinePlotItem(width=3.0, antialias=True)
        self.sensor_line = gl.GLLinePlotItem(width=3.0, antialias=True)
        self.arkit_marker = gl.GLScatterPlotItem(size=9, pxMode=True)
        self.sensor_marker = gl.GLScatterPlotItem(size=9, pxMode=True)
        self.start_marker = gl.GLScatterPlotItem(size=11, pxMode=True)
        self.end_marker = gl.GLScatterPlotItem(size=11, pxMode=True)
        self.error_lines = [gl.GLLinePlotItem(width=1.2, antialias=True) for _ in range(24)]

        for item in [
            self.arkit_line,
            self.sensor_line,
            self.arkit_marker,
            self.sensor_marker,
            self.start_marker,
            self.end_marker,
        ]:
            self.addItem(item)

        for item in self.error_lines:
            self.addItem(item)

    def add_axes(self):
        axes = [
            (np.array([[0, 0, 0], [0.6, 0, 0]], dtype=float), (1.0, 0.2, 0.2, 1.0)),
            (np.array([[0, 0, 0], [0, 0.6, 0]], dtype=float), (0.2, 1.0, 0.2, 1.0)),
            (np.array([[0, 0, 0], [0, 0, 0.6]], dtype=float), (0.2, 0.4, 1.0, 1.0)),
        ]
        for points, color in axes:
            self.addItem(gl.GLLinePlotItem(pos=points, color=color, width=2.0, antialias=True))

    def update_scene(self, arkit_positions, sensor_positions, arkit_pose, sensor_pose, error_segments=None):
        arkit_positions = self.normalized_points(arkit_positions)
        sensor_positions = self.normalized_points(sensor_positions)

        self.update_track(
            self.arkit_line,
            self.arkit_marker,
            arkit_positions,
            line_color=(0.0, 0.85, 1.0, 1.0),
            marker_color=(0.45, 0.95, 1.0, 1.0),
        )
        self.update_track(
            self.sensor_line,
            self.sensor_marker,
            sensor_positions,
            line_color=(1.0, 0.68, 0.12, 0.95),
            marker_color=(1.0, 0.76, 0.22, 1.0),
        )
        self.update_start_end_markers(arkit_positions, sensor_positions)
        self.update_error_segments(error_segments or [])
        self.fit_camera_to_points([arkit_positions, sensor_positions])

    def normalized_points(self, points):
        if points is None or len(points) == 0:
            return np.empty((0, 3), dtype=float)
        points = np.asarray(points, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3:
            return np.empty((0, 3), dtype=float)
        return points

    def update_track(self, line_item, marker_item, points, line_color, marker_color):
        if len(points) > 1:
            line_item.setData(pos=points, color=line_color)
        else:
            line_item.setData(pos=np.empty((0, 3), dtype=float), color=line_color)

        if len(points) > 0:
            marker_item.setData(pos=points[-1:], color=marker_color)
        else:
            marker_item.setData(pos=np.empty((0, 3), dtype=float), color=marker_color)

    def update_start_end_markers(self, arkit_positions, sensor_positions):
        start_points = []
        end_points = []
        if len(arkit_positions) > 0:
            start_points.append(arkit_positions[0])
            end_points.append(arkit_positions[-1])
        if len(sensor_positions) > 0:
            start_points.append(sensor_positions[0])
            end_points.append(sensor_positions[-1])

        self.start_marker.setData(
            pos=np.array(start_points, dtype=float) if start_points else np.empty((0, 3), dtype=float),
            color=(0.2, 1.0, 0.35, 1.0),
        )
        self.end_marker.setData(
            pos=np.array(end_points, dtype=float) if end_points else np.empty((0, 3), dtype=float),
            color=(1.0, 0.25, 0.95, 1.0),
        )

    def update_error_segments(self, error_segments):
        for index, item in enumerate(self.error_lines):
            if index < len(error_segments):
                segment = np.asarray(error_segments[index], dtype=float)
                item.setData(pos=segment, color=(1.0, 0.15, 0.25, 0.45))
            else:
                item.setData(pos=np.empty((0, 3), dtype=float), color=(1.0, 0.15, 0.25, 0.0))

    def fit_camera_to_points(self, point_sets):
        now = time.monotonic()
        if now - self.last_camera_fit < 0.75:
            return

        visible = [points for points in point_sets if len(points) > 0]
        if not visible:
            return

        all_points = np.vstack(visible)
        extent = np.ptp(all_points, axis=0)
        distance = float(max(np.max(extent) * 2.8, 0.35))
        distance = min(distance, 12.0)
        self.setCameraPosition(distance=distance, elevation=28, azimuth=42)
        self.last_camera_fit = now


def camera_frustum_points(sample, scale=0.18):
    rotation = quaternion_to_matrix(sample.quaternion)
    origin = sample.position
    forward = rotation @ np.array([0.0, 1.0, 0.0], dtype=float)
    right = rotation @ np.array([1.0, 0.0, 0.0], dtype=float)
    up = rotation @ np.array([0.0, 0.0, 1.0], dtype=float)

    center = origin + forward * scale
    corners = [
        center + right * scale * 0.55 + up * scale * 0.35,
        center - right * scale * 0.55 + up * scale * 0.35,
        center - right * scale * 0.55 - up * scale * 0.35,
        center + right * scale * 0.55 - up * scale * 0.35,
    ]

    segments = []
    for corner in corners:
        segments.extend([origin, corner])
    for index in range(4):
        segments.extend([corners[index], corners[(index + 1) % 4]])
    segments.extend([origin, origin + forward * scale * 1.35])
    return np.array(segments, dtype=float)


class StatsPanel(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout()
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)

        self.arkit_label = QLabel("iPhone CSV: not loaded")
        self.sensor_label = QLabel("Robot CSV: not loaded")
        self.error_label = QLabel("Status: choose both files to run offline validation")
        self.timing_label = QLabel("Timing: waiting")
        self.calibration_label = QLabel("Result: waiting")
        self.legend_label = QLabel("Output: no result yet")

        for label in [
            self.arkit_label,
            self.sensor_label,
            self.error_label,
            self.timing_label,
            self.calibration_label,
            self.legend_label,
        ]:
            label.setWordWrap(True)
            label.setMinimumHeight(0)
            label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
            layout.addWidget(label)

        self.setLayout(layout)

    def update_input(self, stream, filename, sample_count=None):
        suffix = "" if sample_count is None else f" ({sample_count} samples)"
        if stream == "arkit":
            self.arkit_label.setText(f"iPhone CSV: {filename}{suffix}")
        else:
            self.sensor_label.setText(f"Robot CSV: {filename}{suffix}")

    def update_status(self, message):
        self.error_label.setText(f"Status: {message}")

    def update_offline_result(self, result, output_dir):
        self.error_label.setText("Status: validation complete")
        self.timing_label.setText(
            f"Timing: shift {result.time_shift:+.3f} s | pairs {len(result.hand_eye.matched_pairs)}"
        )
        self.calibration_label.setText(
            f"Result: mean {result.evaluation.mean_absolute_error_mm:.3f} mm | "
            f"rmse {result.evaluation.rmse_absolute_error_mm:.3f} mm | "
            f"max {result.evaluation.max_absolute_error_mm:.3f} mm"
        )
        self.legend_label.setText(f"Output folder: {Path(output_dir).name}")

    def reset_offline(self):
        self.arkit_label.setText("iPhone CSV: not loaded")
        self.sensor_label.setText("Robot CSV: not loaded")
        self.error_label.setText("Status: choose both files to run offline validation")
        self.timing_label.setText("Timing: waiting")
        self.calibration_label.setText("Result: waiting")
        self.legend_label.setText("Output: no result yet")

    def update_stream(self, stream, stats):
        text = (
            f"{stream}: {stats['fps']:.1f} fps | packets {stats['packets']} | v{stats.get('protocol_version', 1)} | "
            f"drops {stats['drops']} | latency {stats['latency_ms']:.1f} ms"
        )
        if stream == "arkit":
            self.arkit_label.setText(text)
        else:
            self.sensor_label.setText(text)

    def update_error(self, position_error, angle_error, time_delta):
        if position_error is None:
            self.error_label.setText("Error: waiting for paired samples")
            return

        self.error_label.setText(
            f"Error: position {position_error:.3f} m | angle {angle_error:.2f} deg | "
            f"time delta {time_delta * 1000.0:.1f} ms"
        )

    def update_calibration(self, result, apply_calibration):
        if not result.enabled:
            self.timing_label.setText("Timing: waiting for calibration")
            self.calibration_label.setText("Calibration: waiting for enough paired motion")
            return

        mode = "applied" if apply_calibration else "estimated"
        angle_text = "--" if result.angle_rmse is None else f"{result.angle_rmse:.2f} deg"
        position_text = "--" if result.position_rmse is None else f"{result.position_rmse:.3f} m"
        mean_time_text = "--" if result.mean_time_error is None else f"{result.mean_time_error * 1000.0:+.1f} ms"
        time_rmse_text = "--" if result.time_rmse is None else f"{result.time_rmse * 1000.0:.1f} ms"
        max_time_text = "--" if result.max_time_error is None else f"{result.max_time_error * 1000.0:.1f} ms"

        self.timing_label.setText(
            f"Timing: offset {result.time_offset * 1000.0:+.0f} ms | "
            f"mean residual {mean_time_text} | rmse {time_rmse_text} | max {max_time_text}"
        )
        self.calibration_label.setText(
            f"Calibration {mode}: {result.quality} | dt {result.time_offset * 1000.0:+.0f} ms | "
            f"slope {result.time_slope:.8f} | "
            f"scale {result.scale:.4f} | pos rmse {position_text} | "
            f"angle rmse {angle_text} | pairs {result.pair_count} | "
            f"inliers {result.inlier_ratio:.0%} | motion {result.motion_coverage}"
        )


class ErrorPlotsPanel(QWidget):
    plot_selected = pyqtSignal(str)

    def __init__(self, output_root="offline_calibration_output"):
        super().__init__()
        self.output_root = Path(output_root)
        self.plot_dir = None
        self.image_labels = []

        layout = QVBoxLayout()
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)

        header = QLabel("Result Plots")
        header.setObjectName("panelHeader")

        self.path_label = QLabel("No result plots loaded")
        self.path_label.setObjectName("mutedLabel")
        self.path_label.setWordWrap(True)

        controls = QHBoxLayout()
        controls.setSpacing(8)
        self.refresh_button = QPushButton("Refresh")
        self.choose_button = QPushButton("Choose")
        self.open_button = QPushButton("Open Folder")
        controls.addWidget(self.refresh_button)
        controls.addWidget(self.choose_button)
        controls.addWidget(self.open_button)

        self.scroll_content = QWidget()
        self.scroll_layout = QVBoxLayout()
        self.scroll_layout.setContentsMargins(0, 0, 0, 0)
        self.scroll_layout.setSpacing(12)
        self.scroll_content.setLayout(self.scroll_layout)

        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setWidget(self.scroll_content)

        layout.addWidget(header)
        layout.addWidget(self.path_label)
        layout.addLayout(controls)
        layout.addWidget(self.scroll_area, 1)
        self.setLayout(layout)

        self.refresh_button.clicked.connect(self.load_latest)
        self.choose_button.clicked.connect(self.choose_directory)
        self.open_button.clicked.connect(self.open_directory)

        self.load_latest()

    def load_latest(self):
        latest_dir = find_latest_error_plot_dir(self.output_root)
        self.load_directory(latest_dir)

    def choose_directory(self):
        start_dir = self.plot_dir or self.output_root
        path = QFileDialog.getExistingDirectory(self, "Choose Error Plot Folder", str(start_dir))
        if path:
            self.load_directory(Path(path))

    def open_directory(self):
        if self.plot_dir is None:
            return
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(self.plot_dir.resolve())))

    def load_directory(self, plot_dir):
        self.clear_images()
        self.plot_dir = Path(plot_dir) if plot_dir else None

        if self.plot_dir is None:
            self.path_label.setText("No result plots found under offline_calibration_output")
            self.add_placeholder("Run offline validation first, or choose a folder that contains generated PNG plots.")
            return

        self.path_label.setText(str(self.plot_dir))
        loaded_count = 0
        first_image_path = None
        for title, filename in ERROR_PLOT_FILES:
            image_path = self.plot_dir / filename
            if image_path.exists():
                if first_image_path is None:
                    first_image_path = image_path
                self.add_image(title, image_path)
                loaded_count += 1

        if loaded_count == 0:
            self.add_placeholder("This folder does not contain recognized error plot PNG files.")
        elif first_image_path is not None:
            self.plot_selected.emit(str(first_image_path))

    def clear_images(self):
        while self.scroll_layout.count():
            item = self.scroll_layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()
        self.image_labels = []

    def add_placeholder(self, message):
        label = QLabel(message)
        label.setObjectName("mutedLabel")
        label.setWordWrap(True)
        self.scroll_layout.addWidget(label)
        self.scroll_layout.addStretch()

    def add_image(self, title, image_path):
        container = QGroupBox(title)
        container_layout = QVBoxLayout()
        container_layout.setContentsMargins(8, 10, 8, 8)
        container_layout.setSpacing(6)

        image_label = QLabel()
        image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        image_label.setMinimumHeight(180)
        image_label.setObjectName("plotImage")

        pixmap = QPixmap(str(image_path))
        if pixmap.isNull():
            image_label.setText(f"Could not load {image_path.name}")
        else:
            scaled = pixmap.scaled(
                360,
                240,
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            )
            image_label.setPixmap(scaled)

        file_label = QLabel(image_path.name)
        file_label.setObjectName("mutedLabel")
        file_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        container_layout.addWidget(image_label)
        container_layout.addWidget(file_label)
        container.setLayout(container_layout)
        self.scroll_layout.addWidget(container)
        self.image_labels.append(image_label)

        for clickable in [container, image_label, file_label]:
            clickable.setCursor(Qt.CursorShape.PointingHandCursor)
            clickable.mousePressEvent = lambda event, path=str(image_path): self.plot_selected.emit(path)


class TrajectoryOverlapImageView(QWidget):
    def __init__(self, output_root="offline_calibration_output"):
        super().__init__()
        self.output_root = Path(output_root)
        self.plot_dir = None
        self.image_path = None
        self.original_pixmap = QPixmap()

        layout = QVBoxLayout()
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(10)

        header_row = QHBoxLayout()
        header_row.setSpacing(8)
        self.title_label = QLabel("3D Trajectory Overlap")
        self.title_label.setObjectName("panelHeader")
        self.refresh_button = QPushButton("Refresh")
        self.choose_button = QPushButton("Choose")
        self.open_button = QPushButton("Open Folder")
        header_row.addWidget(self.title_label, 1)
        header_row.addWidget(self.refresh_button)
        header_row.addWidget(self.choose_button)
        header_row.addWidget(self.open_button)

        self.path_label = QLabel("No trajectory overlap plot loaded")
        self.path_label.setObjectName("mutedLabel")
        self.path_label.setWordWrap(True)

        self.image_label = QLabel()
        self.image_label.setObjectName("overlapImage")
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image_label.setMinimumSize(640, 480)

        layout.addLayout(header_row)
        layout.addWidget(self.path_label)
        layout.addWidget(self.image_label, 1)
        self.setLayout(layout)

        self.refresh_button.clicked.connect(self.load_latest)
        self.choose_button.clicked.connect(self.choose_directory)
        self.open_button.clicked.connect(self.open_directory)

        self.load_latest()

    def load_latest(self):
        self.load_directory(find_latest_error_plot_dir(self.output_root))

    def choose_directory(self):
        start_dir = self.plot_dir or self.output_root
        path = QFileDialog.getExistingDirectory(self, "Choose Trajectory Plot Folder", str(start_dir))
        if path:
            self.load_directory(Path(path))

    def open_directory(self):
        if self.plot_dir is None:
            return
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(self.plot_dir.resolve())))

    def load_directory(self, plot_dir):
        self.plot_dir = Path(plot_dir) if plot_dir else None
        image_path = self.plot_dir / "trajectory_overlap_3d.png" if self.plot_dir else None

        if image_path is None or not image_path.exists():
            self.original_pixmap = QPixmap()
            self.path_label.setText("No trajectory_overlap_3d.png found under offline_calibration_output")
            self.image_label.setText("Run offline validation first, or choose a folder containing trajectory_overlap_3d.png.")
            self.image_label.setPixmap(QPixmap())
            return

        self.load_image_path(str(image_path))

    def load_image_path(self, image_path):
        self.image_path = Path(image_path)
        self.plot_dir = self.image_path.parent

        self.original_pixmap = QPixmap(str(self.image_path))
        if self.original_pixmap.isNull():
            self.path_label.setText(str(self.image_path))
            self.image_label.setPixmap(QPixmap())
            self.image_label.setText(f"Could not load {self.image_path.name}")
            return

        self.title_label.setText(self.display_title_for(self.image_path.name))
        self.path_label.setText(str(self.image_path))
        self.update_scaled_pixmap()

    def display_title_for(self, filename):
        for title, plot_filename in ALL_KNOWN_PLOT_FILES:
            if filename == plot_filename:
                return title
        return Path(filename).stem.replace("_", " ").title()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.update_scaled_pixmap()

    def update_scaled_pixmap(self):
        if self.original_pixmap.isNull():
            return

        target_size = self.image_label.size()
        scaled = self.original_pixmap.scaled(
            target_size,
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self.image_label.setPixmap(scaled)


class MainWindow(QMainWindow):
    def __init__(
        self,
        host,
        arkit_port,
        sensor_port,
        pairing_window,
        max_time_offset,
        arkit_csv=None,
        sensor_csv=None,
        output_dir="offline_calibration_output",
        max_pair_delta=0.05,
        smooth_window=11,
        cross_validate_scale=False,
        cv_folds=5,
        skip_world_refinement=False,
    ):
        super().__init__()
        self.host = host
        self.arkit_port = arkit_port
        self.sensor_port = sensor_port
        self.pairing_window = pairing_window
        self.max_time_offset = max_time_offset
        self.output_dir = output_dir
        self.max_pair_delta = max_pair_delta
        self.smooth_window = smooth_window
        self.cross_validate_scale = cross_validate_scale
        self.cv_folds = cv_folds
        self.skip_world_refinement = skip_world_refinement
        self.offline_worker = None
        self.tracks = {"arkit": PoseTrack(), "sensor": PoseTrack()}
        self.arkit_csv = arkit_csv
        self.sensor_csv = sensor_csv

        self.init_ui()
        self.apply_stylesheet()
        self.load_initial_offline_inputs_if_needed()

    def load_initial_offline_inputs_if_needed(self):
        if self.arkit_csv:
            self.load_csv_path("arkit", self.arkit_csv)
        if self.sensor_csv:
            self.load_csv_path("sensor", self.sensor_csv)
        self.run_offline_calibration_if_ready()

    def init_ui(self):
        self.setWindowTitle("iPhone Trajectory Validator")
        self.resize(1680, 900)

        root = QWidget()
        root_layout = QHBoxLayout()
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)

        side = QWidget()
        side.setFixedWidth(360)
        side_layout = QVBoxLayout()
        side_layout.setContentsMargins(14, 14, 14, 14)
        side_layout.setSpacing(14)

        service_box = QGroupBox("Offline Validation")
        service_layout = QVBoxLayout()

        self.load_arkit_csv_button = QPushButton("Load iPhone CSV")
        self.load_sensor_csv_button = QPushButton("Load Robot Arm CSV")
        self.arkit_csv_label = QLabel("iPhone: Not loaded")
        self.sensor_csv_label = QLabel("Robot Arm: Not loaded")
        self.clear_data_button = QPushButton("Clear All Data")
        self.run_calibration_button = QPushButton("Run Offline Validation")
        self.run_calibration_button.setEnabled(False)

        self.load_arkit_csv_button.clicked.connect(self.load_arkit_csv)
        self.load_sensor_csv_button.clicked.connect(self.load_sensor_csv)
        self.clear_data_button.clicked.connect(self.clear_all_data)
        self.run_calibration_button.clicked.connect(self.run_offline_calibration_if_ready)

        self.arkit_csv_label.setWordWrap(True)
        self.sensor_csv_label.setWordWrap(True)
        self.arkit_csv_label.setStyleSheet("font-size: 11px; color: #a0a8b0;")
        self.sensor_csv_label.setStyleSheet("font-size: 11px; color: #a0a8b0;")

        for widget in [
            self.load_arkit_csv_button,
            self.arkit_csv_label,
            self.load_sensor_csv_button,
            self.sensor_csv_label,
            self.run_calibration_button,
            self.clear_data_button,
        ]:
            service_layout.addWidget(widget)

        self.endpoint_label = QLabel("Select both CSV files. Validation plots will be generated automatically.")
        self.endpoint_label.setWordWrap(True)
        service_layout.addWidget(self.endpoint_label)
        service_box.setLayout(service_layout)

        self.stats_panel = StatsPanel()
        stats_scroll = QScrollArea()
        stats_scroll.setWidgetResizable(True)
        stats_scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        stats_scroll.setMinimumHeight(210)
        stats_scroll.setWidget(self.stats_panel)

        side_layout.addWidget(service_box)
        side_layout.addWidget(stats_scroll, 1)
        side.setLayout(side_layout)

        self.view = TrajectoryOverlapImageView()
        self.error_plots_panel = ErrorPlotsPanel()
        self.error_plots_panel.setFixedWidth(420)
        self.error_plots_panel.plot_selected.connect(self.view.load_image_path)

        root_layout.addWidget(side)
        root_layout.addWidget(self.view, 1)
        root_layout.addWidget(self.error_plots_panel)
        root.setLayout(root_layout)
        self.setCentralWidget(root)

    def apply_stylesheet(self):
        self.setStyleSheet(
            """
            QMainWindow, QWidget {
                background-color: #151b1f;
                color: #edf2f4;
                font-size: 13px;
            }
            QGroupBox {
                border: 1px solid #34434c;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 12px;
                font-weight: bold;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 4px;
            }
            QPushButton {
                background-color: #26343c;
                border: 1px solid #41535f;
                border-radius: 7px;
                padding: 9px 12px;
            }
            QPushButton:hover {
                background-color: #31444f;
            }
            QLabel {
                color: #edf2f4;
                line-height: 145%;
                padding: 1px 0;
            }
            QLabel#panelHeader {
                font-size: 16px;
                font-weight: bold;
            }
            QLabel#mutedLabel {
                color: #9ba8b0;
                font-size: 11px;
            }
            QLabel#plotImage {
                background-color: #0f1418;
                border: 1px solid #2b3942;
                border-radius: 6px;
            }
            QLabel#overlapImage {
                background-color: #f8fafc;
                border: 1px solid #33444f;
                border-radius: 8px;
                color: #182027;
            }
            QScrollArea {
                border: none;
                background-color: transparent;
            }
            """
        )

    def load_arkit_csv(self):
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Load iPhone Pose CSV",
            str(Path.home()),
            "CSV Files (*.csv);;All Files (*.*)"
        )
        if not path:
            return
        self.load_csv_path("arkit", path)
        self.run_offline_calibration_if_ready()

    def load_sensor_csv(self):
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Load Robot Arm Pose CSV",
            str(Path.home()),
            "CSV Files (*.csv);;All Files (*.*)"
        )
        if not path:
            return
        self.load_csv_path("sensor", path)
        self.run_offline_calibration_if_ready()

    def load_csv_path(self, stream, path):
        try:
            samples = load_pose_csv(path, stream)
            self.tracks[stream] = PoseTrack()
            for sample in samples:
                self.tracks[stream].append(sample)

            filename = Path(path).name
            if stream == "arkit":
                self.arkit_csv = path
                self.arkit_csv_label.setText(f"iPhone: {filename} ({len(samples)} samples)")
            else:
                self.sensor_csv = path
                self.sensor_csv_label.setText(f"Robot Arm: {filename} ({len(samples)} samples)")

            self.stats_panel.update_input(stream, filename, len(samples))
            self.update_run_button_state()
            return True
        except Exception as exc:
            if stream == "arkit":
                self.arkit_csv = None
                self.arkit_csv_label.setText("iPhone: Error loading file")
            else:
                self.sensor_csv = None
                self.sensor_csv_label.setText("Robot Arm: Error loading file")
            self.stats_panel.update_status(f"could not load {stream} CSV: {exc}")
            self.update_run_button_state()
            return False

    def update_run_button_state(self):
        ready = bool(self.arkit_csv and self.sensor_csv and self.offline_worker is None)
        self.run_calibration_button.setEnabled(ready)

    def run_offline_calibration_if_ready(self):
        if not self.arkit_csv or not self.sensor_csv or self.offline_worker is not None:
            self.update_run_button_state()
            return

        self.run_calibration_button.setEnabled(False)
        self.load_arkit_csv_button.setEnabled(False)
        self.load_sensor_csv_button.setEnabled(False)
        self.clear_data_button.setEnabled(False)
        self.stats_panel.update_status("running offline validation...")

        self.offline_worker = OfflineCalibrationWorker(
            self.arkit_csv,
            self.sensor_csv,
            self.output_dir,
            self.max_pair_delta,
            self.smooth_window,
            self.cross_validate_scale,
            self.cv_folds,
            self.skip_world_refinement,
        )
        self.offline_worker.status_changed.connect(self.stats_panel.update_status)
        self.offline_worker.finished_successfully.connect(self.on_offline_calibration_finished)
        self.offline_worker.failed.connect(self.on_offline_calibration_failed)
        self.offline_worker.finished.connect(self.on_offline_worker_done)
        self.offline_worker.start()

    def on_offline_calibration_finished(self, result, output_dir, result_path):
        output_path = Path(output_dir)
        self.stats_panel.update_offline_result(result, output_dir)
        self.view.load_directory(output_path)
        self.error_plots_panel.load_directory(output_path)
        self.endpoint_label.setText(f"Latest result:\n{Path(result_path).name}\nFolder: {Path(result_path).parent.name}")

    def on_offline_calibration_failed(self, message):
        self.stats_panel.update_status(f"validation failed: {message}")
        self.endpoint_label.setText("Validation failed. Check the selected CSV files and try again.")

    def on_offline_worker_done(self):
        self.offline_worker = None
        self.load_arkit_csv_button.setEnabled(True)
        self.load_sensor_csv_button.setEnabled(True)
        self.clear_data_button.setEnabled(True)
        self.update_run_button_state()

    def clear_all_data(self):
        if self.offline_worker is not None:
            return
        self.tracks["arkit"] = PoseTrack()
        self.tracks["sensor"] = PoseTrack()
        self.arkit_csv = None
        self.sensor_csv = None
        self.arkit_csv_label.setText("iPhone: Not loaded")
        self.sensor_csv_label.setText("Robot Arm: Not loaded")
        self.stats_panel.reset_offline()
        self.endpoint_label.setText("Select both CSV files. Validation plots will be generated automatically.")
        self.update_run_button_state()

    def closeEvent(self, event):
        if self.offline_worker is not None:
            self.offline_worker.wait(5000)
        event.accept()


def main():
    parser = argparse.ArgumentParser(description="Validate ARKit tracking against a wired sensor stream.")
    parser.add_argument(
        "--mode",
        choices=["gui", "offline"],
        default="gui",
        help="Run the GUI validator or the offline trajectory validation pipeline.",
    )
    parser.add_argument("--host", default="0.0.0.0", help="Host/IP to bind UDP sockets to.")
    parser.add_argument("--arkit-port", type=int, default=5555, help="ARKit UDP pose port.")
    parser.add_argument("--sensor-port", type=int, default=5556, help="Wired sensor UDP pose port.")
    parser.add_argument(
        "--pairing-window",
        type=float,
        default=0.05,
        help="Maximum sender timestamp delta for error pairing, in seconds.",
    )
    parser.add_argument(
        "--max-time-offset",
        type=float,
        default=5.0,
        help="Maximum sensor time offset to scan during offline time synchronization, in seconds.",
    )
    parser.add_argument("--arkit-csv", default=None, help="Offline ARKit pose CSV path.")
    parser.add_argument("--sensor-csv", default=None, help="Offline wired sensor pose CSV path.")
    parser.add_argument(
        "--max-pair-delta",
        type=float,
        default=0.05,
        help="Maximum time delta for nearest-neighbor robot/ARKit pose pairing during offline validation, in seconds.",
    )
    parser.add_argument(
        "--smooth-window",
        type=int,
        default=11,
        help="Savitzky-Golay smoothing window for scalar speed estimation in offline mode.",
    )
    parser.add_argument(
        "--output-dir",
        default="offline_calibration_output",
        help="Root directory for offline result JSON and generated plots. Each run gets its own timestamped subfolder.",
    )
    parser.add_argument(
        "--cross-validate-scale",
        action="store_true",
        help="Deprecated; scale is fixed at 1.0 and this option is ignored.",
    )
    parser.add_argument(
        "--cv-folds",
        type=int,
        default=5,
        help="Number of blocked folds used when --cross-validate-scale is enabled.",
    )
    parser.add_argument(
        "--skip-world-refinement",
        action="store_true",
        help="Skip the optional session-level T_base_world refinement step.",
    )
    args = parser.parse_args()

    if args.mode == "offline":
        if not args.arkit_csv or not args.sensor_csv:
            parser.error("--mode offline requires both --arkit-csv and --sensor-csv.")

        pipeline = OfflinePoseCalibrationPipeline(
            max_pair_delta=args.max_pair_delta,
            smooth_window=args.smooth_window,
            cross_validate_scale=args.cross_validate_scale,
            cv_folds=args.cv_folds,
            skip_world_refinement=args.skip_world_refinement,
        )
        result = pipeline.run(args.arkit_csv, args.sensor_csv)
        print_offline_result(result)

        output_dir = prepare_run_output_dir(args.output_dir, args.arkit_csv, args.sensor_csv)
        CalibrationPlotter(output_dir).create_all(result)
        result_path = output_dir / "offline_calibration_result.json"
        result_path.write_text(json.dumps(serialize_offline_result(result), indent=2), encoding="utf-8")
        print(f"results_saved_to: {result_path}")
        return

    app = QApplication(sys.argv)
    window = MainWindow(
        args.host,
        args.arkit_port,
        args.sensor_port,
        args.pairing_window,
        args.max_time_offset,
        arkit_csv=args.arkit_csv,
        sensor_csv=args.sensor_csv,
    )
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
