"""Real-time COCO-17 2D to Human3.6M 3D lifting with VideoPose3D.

The temporal network is adapted from facebookresearch/VideoPose3D (CC BY-NC 4.0).
This version uses only the single-output inference path and fills unavailable future
frames with the latest observation for low-latency streaming.
"""

from __future__ import annotations

from collections import deque
from pathlib import Path
from typing import Optional, Sequence, Tuple

import numpy as np
import torch
import torch.nn as nn

from motion_packet import Detection


Pose3D = Tuple[Tuple[float, float, float], ...]
COCO_LEFT = (1, 3, 5, 7, 9, 11, 13, 15)
COCO_RIGHT = (2, 4, 6, 8, 10, 12, 14, 16)
H36M_LEFT = (4, 5, 6, 11, 12, 13)
H36M_RIGHT = (1, 2, 3, 14, 15, 16)


class _TemporalModelOptimized1f(nn.Module):
    """The single-output VideoPose3D network used by the official checkpoint."""

    def __init__(self) -> None:
        super().__init__()
        channels = 1024
        widths = (3, 3, 3, 3, 3)
        self.drop = nn.Dropout(0.25)
        self.relu = nn.ReLU(inplace=True)
        self.expand_conv = nn.Conv1d(34, channels, 3, stride=3, bias=False)
        self.expand_bn = nn.BatchNorm1d(channels, momentum=0.1)
        self.shrink = nn.Conv1d(channels, 51, 1)
        self.layers_conv = nn.ModuleList()
        self.layers_bn = nn.ModuleList()
        for width in widths[1:]:
            self.layers_conv.append(nn.Conv1d(channels, channels, width, stride=width, bias=False))
            self.layers_bn.append(nn.BatchNorm1d(channels, momentum=0.1))
            self.layers_conv.append(nn.Conv1d(channels, channels, 1, bias=False))
            self.layers_bn.append(nn.BatchNorm1d(channels, momentum=0.1))

    def forward(self, value: torch.Tensor) -> torch.Tensor:
        batch = value.shape[0]
        value = value.reshape(batch, value.shape[1], 34).permute(0, 2, 1)
        value = self.drop(self.relu(self.expand_bn(self.expand_conv(value))))
        for index in range(4):
            width = self.layers_conv[index * 2].kernel_size[0]
            residual = value[:, :, width // 2 :: width]
            value = self.drop(
                self.relu(self.layers_bn[index * 2](self.layers_conv[index * 2](value)))
            )
            value = residual + self.drop(
                self.relu(
                    self.layers_bn[index * 2 + 1](self.layers_conv[index * 2 + 1](value))
                )
            )
        value = self.shrink(value).permute(0, 2, 1)
        return value.reshape(batch, -1, 17, 3)


class VideoPose3DLifter:
    """Lift the latest DMV1 body while using available past frames as context."""

    receptive_field = 243
    past_frames = receptive_field // 2

    def __init__(self, model_path: str, confidence: float = 0.25) -> None:
        path = Path(model_path)
        if not path.is_file():
            raise FileNotFoundError("VideoPose3D checkpoint not found: {}".format(path))
        self.confidence = confidence
        self._history = deque(maxlen=self.past_frames + 1)
        self._last_points: Optional[np.ndarray] = None
        self._model = _TemporalModelOptimized1f()
        checkpoint = torch.load(path, map_location="cpu", weights_only=False)
        self._model.load_state_dict(checkpoint["model_pos"])
        self._model.eval()

    def _normalize(self, body: Detection, width: int, height: int) -> np.ndarray:
        points = np.asarray(body.keypoints, dtype=np.float32)
        xy = points[:, :2].copy()
        if self._last_points is not None:
            missing = points[:, 2] < self.confidence
            xy[missing] = self._last_points[missing]
        self._last_points = xy.copy()

        # VideoPose3D maps image x to [-1, 1] and preserves the image aspect ratio on y.
        xy[:, 0] = xy[:, 0] * 2.0 - 1.0
        xy[:, 1] = (xy[:, 1] * 2.0 - 1.0) * float(height) / float(width)
        return xy

    def update(self, body: Optional[Detection], width: int, height: int) -> Optional[Pose3D]:
        if body is None:
            return None
        current = self._normalize(body, width, height)
        self._history.append(current)

        history = list(self._history)
        left = [history[0]] * (self.past_frames + 1 - len(history))
        right = [current] * self.past_frames
        window = np.asarray(left + history + right, dtype=np.float32)
        flipped = window.copy()
        flipped[:, :, 0] *= -1.0
        flipped[:, COCO_LEFT + COCO_RIGHT] = flipped[:, COCO_RIGHT + COCO_LEFT]
        with torch.inference_mode():
            output = self._model(torch.from_numpy(np.stack((window, flipped))))[:, 0]
            output[1, :, 0] *= -1.0
            output[1, H36M_LEFT + H36M_RIGHT] = output[1, H36M_RIGHT + H36M_LEFT].clone()
            output = output.mean(dim=0).numpy()
        return tuple(tuple(float(axis) for axis in joint) for joint in output)
