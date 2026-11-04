"""Detect a "pat the head" gesture from DMV1 motion plus FNG1 hand keypoints.

The detector runs entirely on the PC, so the board runtime and its
cross-compiled binary never change.

A real hand near the performer's own head stands in for "patting the digital
human's head"; the nose is the most reliably detected head landmark, so it is
used as the anchor and the test region is shifted upward, toward the crown.

The detector publishes a *state* (0/1) rather than an edge event.  Unity can
therefore consume it as a one-shot trigger today and as a sustained bool later
without any change in this file.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

from finger_packet import FingerFrame
from motion_packet import MotionFrame

# COCO-17 body keypoints.
BODY_COUNT = 17
BODY_NOSE = 0
BODY_LEFT_EAR = 3
BODY_RIGHT_EAR = 4

# 51-point inner-face layout: nose tip is 68-point index 30 -> v1 index 13.
FACE_NOSE_TIP = 13

# Hand landmarks follow the MediaPipe order used by FingerSolver.
HAND_COUNT = 21
HAND_WRIST = 0
HAND_PALM = 9


@dataclass(frozen=True)
class HeadPatState:
    """One frame of detector output; ``active`` is the signal bit."""

    active: int
    progress: float
    dwell: float
    outside: Optional[float]
    head_width_px: float
    nose_x: float = 0.0
    nose_y: float = 0.0
    nose_source: str = "none"
    nose_cached: bool = False
    hand_index: int = -1

    def status(self) -> str:
        outside = "none" if self.outside is None else "{:.2f}".format(self.outside)
        return (
            "pat_active={} pat_progress={:.2f} pat_dwell={:.2f}s pat_outside={} "
            "pat_head={:.0f}px pat_nose={}{} pat_hand={}".format(
                self.active,
                self.progress,
                self.dwell,
                outside,
                self.head_width_px,
                self.nose_source,
                " (cached)" if self.nose_cached else "",
                self.hand_index,
            )
        )


class HeadPatTrigger:
    """Latched "hand pats the head" detector with hysteresis and dropout grace.

    Region geometry is expressed in head widths so the thresholds do not have
    to be re-calibrated when the performer moves closer to or further from the
    camera.  ``region_above`` extends upward from the nose toward the crown,
    which is where a patting hand actually lands.
    """

    def __init__(
        self,
        name: str = "touch_head",
        dwell_seconds: float = 1.0,
        grace_seconds: float = 0.25,
        release_seconds: float = 0.3,
        cooldown_seconds: float = 0.5,
        min_confidence: float = 0.3,
        nose_cache_seconds: float = 0.5,
        region_half_width: float = 0.8,
        region_above: float = 1.3,
        region_below: float = 0.2,
        exit_scale: float = 1.35,
        head_width_fallback: float = 0.20,
        head_width_cache_seconds: float = 3.0,
        head_width_smoothing: float = 0.35,
        whole_hand: bool = True,
    ) -> None:
        self.name = name
        self.dwell_seconds = float(dwell_seconds)
        self.grace_seconds = float(grace_seconds)
        self.release_seconds = float(release_seconds)
        self.cooldown_seconds = float(cooldown_seconds)
        self.min_confidence = float(min_confidence)
        self.nose_cache_seconds = float(nose_cache_seconds)
        self.region_half_width = float(region_half_width)
        self.region_above = float(region_above)
        self.region_below = float(region_below)
        self.exit_scale = float(exit_scale)
        self.head_width_fallback = float(head_width_fallback)
        self.head_width_cache_seconds = float(head_width_cache_seconds)
        self.head_width_smoothing = float(head_width_smoothing)
        self.whole_hand = bool(whole_hand)

        self.active = False
        self.dwell = 0.0
        self._start: Optional[float] = None
        self._last_inside: Optional[float] = None
        self._armed_at = 0.0
        # Cached anchors; the _cache suffix keeps them from shadowing the
        # same-named methods (_nose, _head_width).
        self._nose_cache: Optional[Tuple[float, float]] = None
        self._nose_source = "none"
        self._nose_stamp = 0.0
        self._head_cache: Optional[float] = None
        self._head_stamp = 0.0

    # ------------------------------------------------------------------ head

    def _head_width(self, frame: MotionFrame, width: float, height: float, now: float) -> float:
        """Ear-to-ear span, cached while the ears are occluded.

        A fixed fallback is a poor substitute here: the measured span is about
        a tenth of the frame height, so the 0.20 fallback would balloon the
        region by ~60% exactly while the hand is covering the face.  Reusing
        the last good measurement keeps the region tied to the real head size.
        """
        body = frame.body
        if body is not None and len(body.keypoints) >= BODY_COUNT:
            left = body.keypoints[BODY_LEFT_EAR]
            right = body.keypoints[BODY_RIGHT_EAR]
            if left[2] >= self.min_confidence and right[2] >= self.min_confidence:
                dx = (left[0] - right[0]) * width
                dy = (left[1] - right[1]) * height
                span = math.hypot(dx, dy)
                if 0.05 * height <= span <= 0.60 * height:
                    # Light EMA: raw spans jitter by tens of pixels, and the
                    # region size should not follow that noise.
                    if self._head_cache is None:
                        self._head_cache = span
                    else:
                        alpha = self.head_width_smoothing
                        self._head_cache += alpha * (span - self._head_cache)
                    self._head_stamp = now
                    return self._head_cache

        if (
            self._head_cache is not None
            and (now - self._head_stamp) <= self.head_width_cache_seconds
        ):
            return self._head_cache
        return self.head_width_fallback * height

    def _nose(self, frame: MotionFrame, now: float) -> Tuple[Optional[Tuple[float, float]], str, bool]:
        """Prefer the pose nose, fall back to the face nose tip, then to cache.

        A patting hand occludes the face, so a short-lived cache keeps the
        anchor stable exactly while the gesture is being performed.
        """
        best: Optional[Tuple[float, float, float]] = None
        source = "none"
        if frame.body is not None and len(frame.body.keypoints) > BODY_NOSE:
            point = frame.body.keypoints[BODY_NOSE]
            if point[2] >= self.min_confidence:
                best = point
                source = "body"
        if frame.face is not None and len(frame.face.keypoints) > FACE_NOSE_TIP:
            point = frame.face.keypoints[FACE_NOSE_TIP]
            if point[2] >= self.min_confidence and (best is None or point[2] > best[2]):
                best = point
                source = "face"

        if best is not None:
            self._nose_cache = (best[0], best[1])
            self._nose_source = source
            self._nose_stamp = now
            return self._nose_cache, source, False

        if self._nose_cache is not None and (now - self._nose_stamp) <= self.nose_cache_seconds:
            return self._nose_cache, self._nose_source, True
        return None, "none", False

    # ---------------------------------------------------------------- region

    def _outside(
        self,
        px: float,
        py: float,
        nose_x: float,
        nose_y: float,
        head_width: float,
        scale: float,
    ) -> float:
        """Scale-free distance outside the head region; 0.0 means inside."""
        half_width = self.region_half_width * head_width * scale
        top = nose_y - self.region_above * head_width * scale
        bottom = nose_y + self.region_below * head_width * scale
        dx = max(0.0, abs(px - nose_x) - half_width)
        dy = max(0.0, top - py, py - bottom)
        return math.hypot(dx, dy) / head_width

    def _hand(
        self,
        finger_frame: Optional[FingerFrame],
        nose_x: float,
        nose_y: float,
        head_width: float,
        width: float,
        height: float,
    ) -> Tuple[Optional[float], Optional[float], int]:
        """Return (enter metric, exit metric, hand index) for the closest hand."""
        if finger_frame is None:
            return None, None, -1
        best_enter = None
        best_exit = None
        best_index = -1
        for index, hand in enumerate(finger_frame.hands):
            if len(hand.keypoints) < HAND_COUNT or hand.score < self.min_confidence:
                continue
            wrist = hand.keypoints[HAND_WRIST]
            palm = hand.keypoints[HAND_PALM]
            if wrist[2] < self.min_confidence or palm[2] < self.min_confidence:
                continue
            points = (wrist, palm) if self.whole_hand else (palm,)
            enter = max(
                self._outside(p[0] * width, p[1] * height, nose_x, nose_y, head_width, 1.0)
                for p in points
            )
            exit_metric = min(
                self._outside(
                    p[0] * width, p[1] * height, nose_x, nose_y, head_width, self.exit_scale
                )
                for p in points
            )
            if best_enter is None or enter < best_enter:
                best_enter = enter
                best_exit = exit_metric
                best_index = index
        return best_enter, best_exit, best_index

    # ----------------------------------------------------------------- state

    def _progress(self) -> float:
        if self.dwell_seconds <= 0.0:
            return 0.0
        return min(1.0, self.dwell / self.dwell_seconds)

    def update(
        self,
        frame: MotionFrame,
        finger_frame: Optional[FingerFrame],
        now: Optional[float] = None,
    ) -> HeadPatState:
        """Advance the detector by one frame and return the current signal bit."""
        now = time.monotonic() if now is None else now
        width = float(frame.frame_width) or 1920.0
        height = float(frame.frame_height) or 1080.0
        head_width = self._head_width(frame, width, height, now)
        nose, source, cached = self._nose(frame, now)
        if nose is None:
            return self._miss(head_width, now)

        nose_x = nose[0] * width
        nose_y = nose[1] * height
        enter, exit_metric, hand_index = self._hand(
            finger_frame, nose_x, nose_y, head_width, width, height
        )
        if enter is None:
            return self._miss(head_width, now, nose_x, nose_y, source, cached)

        outside = enter if not self.active else exit_metric
        assert outside is not None
        in_zone = outside <= 0.0
        self._advance(now, in_zone)
        return HeadPatState(
            active=1 if self.active else 0,
            progress=self._progress(),
            dwell=self.dwell,
            outside=outside,
            head_width_px=head_width,
            nose_x=nose_x,
            nose_y=nose_y,
            nose_source=source,
            nose_cached=cached,
            hand_index=hand_index,
        )

    def _miss(
        self,
        head_width: float,
        now: float,
        nose_x: float = 0.0,
        nose_y: float = 0.0,
        source: str = "none",
        cached: bool = False,
    ) -> HeadPatState:
        """No usable hand or nose this frame; grace still keeps the dwell alive."""
        self._advance(now, False)
        return HeadPatState(
            active=1 if self.active else 0,
            progress=self._progress(),
            dwell=self.dwell,
            outside=None,
            head_width_px=head_width,
            nose_x=nose_x,
            nose_y=nose_y,
            nose_source=source,
            nose_cached=cached,
        )

    def _advance(self, now: float, in_zone: bool) -> None:
        """Wall-clock dwell, so a single late frame cannot wipe out progress."""
        if self.active:
            if in_zone:
                self._last_inside = now
            elif self._last_inside is not None and now - self._last_inside >= self.release_seconds:
                self.active = False
                self._armed_at = now
                self._start = None
                self._last_inside = None
                self.dwell = 0.0
            return

        if in_zone:
            if self._start is None:
                self._start = now
            self._last_inside = now
            self.dwell = now - self._start
            if self.dwell >= self.dwell_seconds and now - self._armed_at >= self.cooldown_seconds:
                self.active = True
            return

        # Outside the region, or no usable hand: only a gap longer than the
        # grace period abandons the gesture in progress.
        if (
            self._start is not None
            and self._last_inside is not None
            and now - self._last_inside > self.grace_seconds
        ):
            self._start = None
            self._last_inside = None
            self.dwell = 0.0


class HeadPatJsonlLogger:
    """Per-frame calibration log; every line is a complete JSON record."""

    def __init__(self, path: str) -> None:
        output = Path(path).expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        self.path = output
        self._file = output.open("w", encoding="utf-8")

    def write(self, sequence: int, source: str, state: HeadPatState) -> None:
        record = {
            "receive_time_ns": time.time_ns(),
            "sequence": sequence,
            "source": source,
            "active": state.active,
            "progress": round(state.progress, 4),
            "dwell": round(state.dwell, 4),
            "outside": None if state.outside is None else round(state.outside, 4),
            "head_width_px": round(state.head_width_px, 2),
            "nose_x_px": round(state.nose_x, 2),
            "nose_y_px": round(state.nose_y, 2),
            "nose_source": state.nose_source,
            "nose_cached": state.nose_cached,
            "hand_index": state.hand_index,
        }
        self._file.write(json.dumps(record, separators=(",", ":")) + "\n")
        self._file.flush()

    def close(self) -> None:
        self._file.close()
