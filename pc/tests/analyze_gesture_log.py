"""Summarise a --gesture-log JSONL file and recommend threshold adjustments.

Run after one calibration session:

    python analyze_gesture_log.py ..\\logs\\gesture_debug.jsonl

The useful number is ``pat_outside``: how far the hand is from the head region,
measured in head widths.  0.00 means inside.  If its minimum never gets close
to 0 while you are patting your head, the region is in the wrong place or too
small, and no amount of waiting will ever trigger the signal.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path


def load(path: Path):
    rows = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except ValueError:
                continue
    return rows


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(round(fraction * (len(ordered) - 1)))))
    return ordered[index]


def longest_inside(rows):
    """Longest wall-clock stretch with outside <= 0, using logged timestamps."""
    best = 0.0
    current_start = None
    previous = None
    for row in rows:
        stamp = row.get("receive_time_ns", 0) / 1e9
        outside = row.get("outside")
        inside = outside is not None and outside <= 0.0
        if inside:
            if current_start is None:
                current_start = stamp
            elif previous is not None:
                # A gap longer than the detector grace breaks the streak.
                if stamp - previous > 0.25:
                    best = max(best, previous - current_start)
                    current_start = stamp
            previous = stamp
        else:
            if current_start is not None:
                best = max(best, previous - current_start if previous else 0.0)
                current_start = None
            previous = None
    if current_start is not None and previous is not None:
        best = max(best, previous - current_start)
    return best


def main() -> int:
    # Windows 控制台默认是 GBK，管道重定向时输出中文会崩；统一成 UTF-8。
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass
    if len(sys.argv) > 1:
        path = Path(sys.argv[1])
    else:
        path = Path(__file__).resolve().parents[1] / "logs" / "gesture_debug.jsonl"
    if not path.is_file():
        print("找不到日志文件: {}".format(path))
        print("先跑一次：python motion_bridge.py --gesture --gesture-log <路径>")
        return 1

    rows = load(path)
    if not rows:
        print("日志是空的: {}".format(path))
        return 1

    span = (rows[-1].get("receive_time_ns", 0) - rows[0].get("receive_time_ns", 0)) / 1e9
    outside_values = [r["outside"] for r in rows if r.get("outside") is not None]
    blind = len(rows) - len(outside_values)
    sources = {}
    for row in rows:
        key = row.get("nose_source", "?")
        sources[key] = sources.get(key, 0) + 1
    cached = sum(1 for row in rows if row.get("nose_cached"))
    heads = [r.get("head_width_px", 0.0) for r in rows if r.get("head_width_px")]
    triggers = sum(
        1
        for previous, row in zip(rows, rows[1:])
        if row.get("active") == 1 and previous.get("active") == 0
    )

    print("日志: {}".format(path))
    print("帧数: {}   时长: {:.1f}s   平均帧率: {:.1f} fps".format(len(rows), span, len(rows) / span if span else 0.0))
    print("无法判定帧（鼻子或手不可用）: {} ({:.1f}%)".format(blind, 100.0 * blind / len(rows)))
    print("鼻子来源: {}   使用缓存: {} ({:.1f}%)".format(
        ", ".join("{}={}".format(k, v) for k, v in sorted(sources.items())),
        cached, 100.0 * cached / len(rows)))
    if heads:
        print("头宽(耳间距): min={:.0f}px 中位={:.0f}px max={:.0f}px".format(
            min(heads), percentile(heads, 0.5), max(heads)))
    print("触发次数: {}".format(triggers))

    if not outside_values:
        print("\n结论: 整段日志都拿不到可用的鼻子或手部数据，手势不可能触发。")
        print("      先确认状态行里 hands= 大于 0、finger_rx 有数据。")
        return 1

    print("\npat_outside 分布（0 表示手在头部区域内）:")
    for label, fraction in (("最小", 0.0), ("p05", 0.05), ("p10", 0.10), ("p25", 0.25), ("中位", 0.50)):
        value = percentile(outside_values, fraction)
        print("  {:<4} {:.3f}".format(label, value))

    best = min(outside_values)
    reach = longest_inside(rows)
    print("\n手最接近区域的时刻: pat_outside={:.3f}".format(best))
    print("连续停留在区域内的最长时间: {:.2f}s".format(reach))

    print("\n建议:")
    if best > 0.35:
        print("  [X] 手从未接近头部区域。区域位置或大小不对，先调 --gesture-above 和 --gesture-width。")
    elif best > 0.05:
        print("  [!] 手最接近时仍差 {:.2f} 个头宽，区域偏远。".format(best))
        print("      摸头动作实际停在偏上/偏侧的位置时，调大 --gesture-width（如 1.0），")
        print("      或按实际落点调 --gesture-above（越大区域越靠头顶，默认 1.3）。")
    else:
        print("  [OK] 手能进入区域，区域位置合理。")
    if reach < 1.0:
        print("  [X] 最长连续停留只有 {:.2f}s，达不到 1.0s 的停留要求。".format(reach))
        print("      要么放慢动作/停稳一点，要么降低 --gesture-dwell（如 0.6）。")
    else:
        print("  [OK] 连续停留 {:.2f}s，满足 1.0s 的停留要求。".format(reach))
    if cached > 0.4 * len(rows):
        print("  [!] 有 {:.0f}% 的帧在用缓存的鼻子位置，遮挡较严重。".format(100.0 * cached / len(rows)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
