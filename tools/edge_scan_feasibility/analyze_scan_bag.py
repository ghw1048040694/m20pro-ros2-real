#!/usr/bin/env python3
"""Analyze LaserScan topics in a rosbag2 bag."""

from __future__ import annotations

import argparse
import json
import math
import sqlite3
import statistics
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import LaserScan


@dataclass
class ScanStats:
    samples: int = 0
    unique_stamps: set = field(default_factory=set)
    first_bag_time: Optional[float] = None
    last_bag_time: Optional[float] = None
    frames: Dict[str, int] = field(default_factory=lambda: defaultdict(int))
    finite_bins: List[int] = field(default_factory=list)
    ages: List[float] = field(default_factory=list)
    intervals: List[float] = field(default_factory=list)
    last_message_time: Optional[float] = None
    range_min: Optional[float] = None
    range_max: Optional[float] = None
    angle_min: Optional[float] = None
    angle_max: Optional[float] = None
    angle_increment: Optional[float] = None

    def add(self, msg: LaserScan, bag_time_s: float) -> None:
        if self.first_bag_time is None:
            self.first_bag_time = bag_time_s
        if self.last_message_time is not None:
            interval = bag_time_s - self.last_message_time
            if interval >= 0:
                self.intervals.append(interval)
        self.last_message_time = bag_time_s
        self.last_bag_time = bag_time_s
        self.samples += 1

        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.unique_stamps.add((msg.header.stamp.sec, msg.header.stamp.nanosec))
        if stamp_s > 1.0:
            self.ages.append(max(0.0, bag_time_s - stamp_s))
        self.frames[msg.header.frame_id or "<empty>"] += 1
        self.finite_bins.append(sum(1 for value in msg.ranges if math.isfinite(value)))
        self.range_min = float(msg.range_min)
        self.range_max = float(msg.range_max)
        self.angle_min = float(msg.angle_min)
        self.angle_max = float(msg.angle_max)
        self.angle_increment = float(msg.angle_increment)

    @staticmethod
    def _summary(values: List[float]) -> Dict[str, Optional[float]]:
        if not values:
            return {"min": None, "mean": None, "p95": None, "max": None}
        sorted_values = sorted(values)
        p95_idx = min(len(sorted_values) - 1, int(round(0.95 * (len(sorted_values) - 1))))
        return {
            "min": sorted_values[0],
            "mean": statistics.fmean(values),
            "p95": sorted_values[p95_idx],
            "max": sorted_values[-1],
        }

    def summary(self) -> Dict[str, object]:
        duration = 0.0
        if self.first_bag_time is not None and self.last_bag_time is not None:
            duration = max(0.0, self.last_bag_time - self.first_bag_time)
        unique = len(self.unique_stamps)
        rate = unique / duration if duration > 0.0 else 0.0
        finite_summary = self._summary([float(value) for value in self.finite_bins])
        return {
            "samples": self.samples,
            "unique": unique,
            "duration_s": duration,
            "rate_hz": rate,
            "frames": dict(self.frames),
            "finite": finite_summary,
            "age": self._summary(self.ages),
            "interval": self._summary(self.intervals),
            "range_min": self.range_min,
            "range_max": self.range_max,
            "angle_min": self.angle_min,
            "angle_max": self.angle_max,
            "angle_increment": self.angle_increment,
        }


def resolve_bag_path(path_text: str) -> Path:
    path = Path(path_text).expanduser().resolve()
    return path


def open_reader(uri: str) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=uri, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)
    return reader


def topic_type_map(reader: rosbag2_py.SequentialReader) -> Dict[str, str]:
    return {topic.name: topic.type for topic in reader.get_all_topics_and_types()}


def analyze_bag(uri: str, topics: Iterable[str]) -> Dict[str, Dict[str, object]]:
    wanted = set(topics)
    reader = open_reader(uri)
    types = topic_type_map(reader)
    stats = {topic: ScanStats() for topic in wanted}
    message_types = {}
    for topic in wanted:
        topic_type = types.get(topic)
        if topic_type:
            message_types[topic] = get_message(topic_type)

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if topic not in wanted:
            continue
        msg_type = message_types.get(topic)
        if msg_type is None:
            continue
        if msg_type is not LaserScan:
            continue
        msg = deserialize_message(data, msg_type)
        stats[topic].add(msg, timestamp_ns * 1e-9)

    return {topic: stats[topic].summary() for topic in topics}


def analyze_db3(db_path: Path, topics: Iterable[str]) -> Dict[str, Dict[str, object]]:
    wanted = list(topics)
    stats = {topic: ScanStats() for topic in wanted}
    connection = sqlite3.connect(str(db_path))
    try:
        rows = connection.execute("select id, name, type from topics").fetchall()
        topic_meta = {int(row[0]): (str(row[1]), str(row[2])) for row in rows}
        wanted_ids = {
            topic_id: (name, topic_type)
            for topic_id, (name, topic_type) in topic_meta.items()
            if name in stats and topic_type == "sensor_msgs/msg/LaserScan"
        }
        if not wanted_ids:
            return {topic: stats[topic].summary() for topic in wanted}
        query = (
            "select topic_id, timestamp, data from messages "
            f"where topic_id in ({','.join('?' for _ in wanted_ids)}) "
            "order by timestamp"
        )
        laser_scan_type = get_message("sensor_msgs/msg/LaserScan")
        for topic_id, timestamp_ns, data in connection.execute(query, list(wanted_ids.keys())):
            topic_name, _topic_type = wanted_ids[int(topic_id)]
            msg = deserialize_message(bytes(data), laser_scan_type)
            stats[topic_name].add(msg, int(timestamp_ns) * 1e-9)
    finally:
        connection.close()
    return {topic: stats[topic].summary() for topic in wanted}


def print_text(results: Dict[str, Dict[str, object]]) -> None:
    for topic, summary in results.items():
        print(f"RESULT {topic}")
        for key, value in summary.items():
            print(f"  {key}: {value}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", help="rosbag2 directory or .db3 file")
    parser.add_argument(
        "--topics",
        nargs="+",
        default=["/scan", "/m20pro/scan_edge_exp"],
        help="LaserScan topics to analyze",
    )
    parser.add_argument("--json", action="store_true", help="print JSON output")
    parser.add_argument(
        "--allow-missing",
        action="store_true",
        help="exit 0 even if one of the requested topics has no samples",
    )
    args = parser.parse_args()

    bag_path = resolve_bag_path(args.bag)
    if bag_path.is_file() and bag_path.suffix == ".db3":
        results = analyze_db3(bag_path, args.topics)
    else:
        results = analyze_bag(str(bag_path), args.topics)
    if args.json:
        print(json.dumps(results, ensure_ascii=False, indent=2))
    else:
        print_text(results)

    missing = [topic for topic, summary in results.items() if int(summary.get("samples", 0)) <= 0]
    if missing and not args.allow_missing:
        print("MISSING: " + ", ".join(missing))
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
