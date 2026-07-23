#!/usr/bin/env python3
"""Offline tests for terrain_guard replay summaries."""

from __future__ import annotations

import sys
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_navigation"))

from m20pro_navigation.terrain_guard_replay import (  # noqa: E402
    load_json_frames,
    replay_frames,
)


def request() -> dict:
    return {
        "request_id": "offline-replay-1",
        "route_id": "stairs-a-up",
        "plan_id": "offline-replay-plan",
        "map_epoch": 1,
        "corridor_version": "corridor-v1",
        "direction": "forward",
        "corridor": {
            "width_m": 1.0,
            "lookahead_m": 1.2,
            "bin_size_m": 0.2,
            "min_step_height_m": 0.05,
            "max_step_height_m": 0.24,
            "obstacle_height_m": 0.22,
            "min_points_per_bin": 4,
            "min_step_count": 2,
            "min_coverage": 0.55,
        },
    }


def cloud(
    levels: list[float],
    *,
    lateral: tuple[float, ...] = (-0.3, -0.1, 0.1, 0.3),
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for index, level in enumerate(levels):
        x = index * 0.2 + 0.05
        points.extend((x, side, level) for side in lateral)
    return points


def test_replay_summary_tracks_states_and_transitions() -> None:
    summary = replay_frames(
        [
            {"stamp_s": 1.0, "points": cloud([0.0, 0.0, 0.12, 0.12, 0.24, 0.24])},
            {
                "stamp_s": 2.0,
                "points": cloud(
                    [0.0, 0.0, 0.12, 0.12, 0.24, 0.24],
                    lateral=(-0.05, 0.0, 0.05),
                ),
            },
            {"stamp_s": 3.0, "points": cloud([0.0, 0.4, 0.4, 0.4, 0.4, 0.4])},
            {"stamp_s": 4.0, "cloud_age_s": 1.0, "points": cloud([0.0] * 6)},
        ],
        request(),
    )
    assert summary["frames"] == 4
    assert summary["states"] == {"traversable": 1, "unknown": 1, "blocked": 1, "stale": 1}
    assert len(summary["transitions"]) == 3
    assert summary["first_stamp_s"] == 1.0
    assert summary["last_stamp_s"] == 4.0
    assert summary["mean_evaluation_ms"] >= 0.0


def test_replay_max_frames_is_bounded() -> None:
    summary = replay_frames(
        [{"points": cloud([0.0] * 6)} for _ in range(5)],
        request(),
        max_frames=2,
    )
    assert summary["frames"] == 2
    assert len(summary["records"]) == 2


def test_jsonl_loader_accepts_frame_stream() -> None:
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "frames.jsonl"
        path.write_text(
            '{"stamp_s": 1.0, "points": []}\n{"stamp_s": 2.0, "points": []}\n',
            encoding="utf-8",
        )
        frames = list(load_json_frames(path))
    assert len(frames) == 2
    assert frames[1]["stamp_s"] == 2.0


def test_replay_can_omit_per_frame_records() -> None:
    summary = replay_frames(
        [{"points": cloud([0.0] * 6)} for _ in range(3)],
        request(),
        include_records=False,
    )
    assert summary["frames"] == 3
    assert "records" not in summary


def main() -> int:
    test_replay_summary_tracks_states_and_transitions()
    print("[OK] test_replay_summary_tracks_states_and_transitions")
    test_replay_max_frames_is_bounded()
    print("[OK] test_replay_max_frames_is_bounded")
    test_jsonl_loader_accepts_frame_stream()
    print("[OK] test_jsonl_loader_accepts_frame_stream")
    test_replay_can_omit_per_frame_records()
    print("[OK] test_replay_can_omit_per_frame_records")
    print("[OK] terrain guard replay tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
