#!/usr/bin/env python3
"""Offline replay for the read-only terrain guard contract.

The replay path never creates a ROS node and never publishes a command. It can
consume a rosbag2 PointCloud2 topic when ROS bag libraries are available, or a
JSON/JSONL fixture so the classifier can be tested without ROS middleware.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from collections import Counter
from pathlib import Path
from typing import Any, Dict, Iterable, Iterator, List, Optional

from .terrain_guard_contract import inspect_cloud


def _finite(value: Any) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return number if math.isfinite(number) else None


def replay_frames(
    frames: Iterable[Any],
    request: Dict[str, Any],
    *,
    cloud_timeout_s: float = 0.75,
    max_frames: int = 0,
    include_records: bool = True,
) -> Dict[str, Any]:
    """Evaluate frames and return bounded state/reason/timing statistics."""
    records: List[Dict[str, Any]] = []
    states: Counter[str] = Counter()
    reasons: Counter[str] = Counter()
    transitions: List[Dict[str, Any]] = []
    previous_state: Optional[str] = None
    frame_count = 0
    stamps: List[float] = []
    started = time.perf_counter()

    for index, frame in enumerate(frames):
        if max_frames > 0 and index >= max_frames:
            break
        frame_count += 1
        if not isinstance(frame, dict):
            frame = {}
        points = frame.get("points") if isinstance(frame.get("points"), list) else []
        result = inspect_cloud(
            points,
            request=request,
            cloud_age_s=frame.get("cloud_age_s", 0.0),
            cloud_timeout_s=cloud_timeout_s,
        )
        state = str(result.get("state") or "unknown")
        reason = str(result.get("reason") or "unknown")
        states[state] += 1
        reasons[reason] += 1
        if previous_state is not None and state != previous_state:
            transitions.append({"frame": index, "from": previous_state, "to": state})
        previous_state = state
        record = dict(result)
        record["frame"] = index
        if frame.get("stamp_s") is not None:
            record["stamp_s"] = frame.get("stamp_s")
            stamp = _finite(frame.get("stamp_s"))
            if stamp is not None:
                stamps.append(stamp)
        if include_records:
            records.append(record)

    elapsed = time.perf_counter() - started
    summary: Dict[str, Any] = {
        "frames": frame_count,
        "states": dict(states),
        "reasons": dict(reasons),
        "transitions": transitions,
        "first_stamp_s": min(stamps) if stamps else None,
        "last_stamp_s": max(stamps) if stamps else None,
        "traversable_ratio": states.get("traversable", 0) / frame_count if frame_count else 0.0,
        "blocked_ratio": states.get("blocked", 0) / frame_count if frame_count else 0.0,
        "evaluation_elapsed_s": elapsed,
        "mean_evaluation_ms": elapsed * 1000.0 / frame_count if frame_count else 0.0,
    }
    if include_records:
        summary["records"] = records
    return summary


def load_json_frames(path: Path) -> Iterator[Dict[str, Any]]:
    """Load a JSON array/object or one JSON object per line."""
    text = path.read_text(encoding="utf-8")
    try:
        payload = json.loads(text)
    except json.JSONDecodeError:
        for line_number, line in enumerate(text.splitlines(), start=1):
            if not line.strip():
                continue
            value = json.loads(line)
            if not isinstance(value, dict):
                raise ValueError(f"JSONL line {line_number} is not an object")
            yield value
        return

    if isinstance(payload, dict) and isinstance(payload.get("frames"), list):
        payload = payload["frames"]
    if isinstance(payload, dict):
        payload = [payload]
    if not isinstance(payload, list):
        raise ValueError("replay JSON must be an object, array, or JSONL objects")
    for index, value in enumerate(payload):
        if not isinstance(value, dict):
            raise ValueError(f"replay frame {index} is not an object")
        yield value


def load_rosbag_frames(uri: str, topic: str) -> Iterator[Dict[str, Any]]:
    """Lazily deserialize one PointCloud2 topic from a rosbag2 source."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs_py import point_cloud2
    except ImportError as exc:  # pragma: no cover - depends on ROS host
        raise RuntimeError("rosbag2 replay requires ROS 2 rosbag2_py and sensor_msgs_py") from exc

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        raise ValueError(f"bag does not contain topic {topic}")
    if topic_types[topic] != "sensor_msgs/msg/PointCloud2":
        raise ValueError(f"topic {topic} has type {topic_types[topic]}, not PointCloud2")
    message_type = get_message(topic_types[topic])
    if message_type is not PointCloud2:
        raise ValueError(f"cannot load PointCloud2 type for {topic}")

    while reader.has_next():
        current_topic, data, timestamp_ns = reader.read_next()
        if current_topic != topic:
            continue
        message = deserialize_message(data, message_type)
        points = [
            (float(x), float(y), float(z))
            for x, y, z in point_cloud2.read_points(
                message,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )
        ]
        yield {"stamp_s": timestamp_ns * 1e-9, "cloud_age_s": 0.0, "points": points}


def load_db3_frames(path: Path, topic: str) -> Iterator[Dict[str, Any]]:
    """Read a standalone sqlite3 bag file without requiring a bag directory."""
    import sqlite3

    try:
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs_py import point_cloud2
    except ImportError as exc:  # pragma: no cover - depends on ROS host
        raise RuntimeError("db3 replay requires ROS 2 sensor message libraries") from exc

    connection = sqlite3.connect(str(path))
    try:
        topics = {
            int(topic_id): (str(name), str(topic_type))
            for topic_id, name, topic_type in connection.execute("select id, name, type from topics")
        }
        matches = [
            topic_id
            for topic_id, (name, topic_type) in topics.items()
            if name == topic and topic_type == "sensor_msgs/msg/PointCloud2"
        ]
        if not matches:
            declared = [name for name, _topic_type in topics.values() if name == topic]
            if declared:
                raise ValueError(f"topic {topic} is not PointCloud2")
            raise ValueError(f"bag does not contain topic {topic}")
        topic_id = matches[0]
        for timestamp_ns, data in connection.execute(
            "select timestamp, data from messages where topic_id = ? order by timestamp",
            (topic_id,),
        ):
            message = deserialize_message(bytes(data), PointCloud2)
            points = [
                (float(x), float(y), float(z))
                for x, y, z in point_cloud2.read_points(
                    message,
                    field_names=("x", "y", "z"),
                    skip_nans=True,
                )
            ]
            yield {"stamp_s": int(timestamp_ns) * 1e-9, "cloud_age_s": 0.0, "points": points}
    finally:
        connection.close()


def load_frames(source: Path, topic: str) -> Iterator[Dict[str, Any]]:
    if source.is_file() and source.suffix.lower() != ".db3":
        yield from load_json_frames(source)
        return
    if source.suffix.lower() == ".db3":
        yield from load_db3_frames(source, topic)
        return
    if source.is_dir():
        yield from load_rosbag_frames(str(source), topic)
        return
    raise ValueError(f"replay source does not exist: {source}")


def _load_request(path: Path) -> Dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("request JSON must be an object")
    return payload


def main(args: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Replay m20pro terrain_guard without motion or ROS publication"
    )
    parser.add_argument("source", help="rosbag2 directory/.db3, JSON array, JSON object, or JSONL file")
    parser.add_argument("--request", required=True, help="terrain request JSON")
    parser.add_argument("--topic", default="/LIDAR/POINTS", help="PointCloud2 topic when source is rosbag2")
    parser.add_argument("--cloud-timeout-s", type=float, default=0.75)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--without-records", action="store_true", help="omit per-frame results")
    parser.add_argument("--json", action="store_true", help="print JSON summary")
    options = parser.parse_args(args)

    summary = replay_frames(
        load_frames(Path(options.source).expanduser().resolve(), options.topic),
        _load_request(Path(options.request).expanduser().resolve()),
        cloud_timeout_s=options.cloud_timeout_s,
        max_frames=options.max_frames,
        include_records=not options.without_records,
    )
    if options.json:
        print(json.dumps(summary, ensure_ascii=False, indent=2))
    else:
        print(f"frames: {summary['frames']}")
        print(f"states: {summary['states']}")
        print(f"reasons: {summary['reasons']}")
        print(f"transitions: {summary['transitions']}")
        print(f"mean_evaluation_ms: {summary['mean_evaluation_ms']:.3f}")
    return 0 if summary["frames"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
