#!/usr/bin/env python3
"""Compare two LaserScan topics during 106 edge scan experiments."""

from __future__ import annotations

import argparse
import math
import time
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Dict, List

import rclpy
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


@dataclass
class TopicStats:
    samples: int = 0
    unique_stamps: set = field(default_factory=set)
    first_arrival: float | None = None
    last_arrival: float | None = None
    frames: Dict[str, int] = field(default_factory=lambda: defaultdict(int))
    finite_bins: List[int] = field(default_factory=list)
    ages: List[float] = field(default_factory=list)
    range_min: float | None = None
    range_max: float | None = None
    angle_min: float | None = None
    angle_max: float | None = None
    angle_increment: float | None = None

    def add(self, msg: LaserScan) -> None:
        now = time.time()
        if self.first_arrival is None:
            self.first_arrival = now
        self.last_arrival = now
        self.samples += 1
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.unique_stamps.add((msg.header.stamp.sec, msg.header.stamp.nanosec))
        if stamp > 1.0:
            self.ages.append(max(0.0, now - stamp))
        self.frames[msg.header.frame_id or "<empty>"] += 1
        self.finite_bins.append(sum(1 for value in msg.ranges if math.isfinite(value)))
        self.range_min = msg.range_min
        self.range_max = msg.range_max
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment

    def summary(self) -> dict:
        duration = 0.0
        if self.first_arrival is not None and self.last_arrival is not None:
            duration = max(0.0, self.last_arrival - self.first_arrival)
        unique = len(self.unique_stamps)
        rate = unique / duration if duration > 0 else 0.0
        return {
            "samples": self.samples,
            "unique": unique,
            "rate_hz": rate,
            "frames": dict(self.frames),
            "finite_min": min(self.finite_bins) if self.finite_bins else None,
            "finite_mean": (
                sum(self.finite_bins) / len(self.finite_bins) if self.finite_bins else None
            ),
            "finite_max": max(self.finite_bins) if self.finite_bins else None,
            "age_min": min(self.ages) if self.ages else None,
            "age_mean": sum(self.ages) / len(self.ages) if self.ages else None,
            "age_max": max(self.ages) if self.ages else None,
            "range_min": self.range_min,
            "range_max": self.range_max,
            "angle_min": self.angle_min,
            "angle_max": self.angle_max,
            "angle_increment": self.angle_increment,
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("topics", nargs="+")
    args = parser.parse_args()
    if len(args.topics) < 1:
        parser.error("at least one topic is required")

    rclpy.init()
    node = rclpy.create_node("m20pro_compare_scan_topics")
    qos = QoSProfile(
        depth=50,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )
    stats = {topic: TopicStats() for topic in args.topics}

    for topic in args.topics:
        node.create_subscription(
            LaserScan,
            topic,
            lambda msg, topic=topic: stats[topic].add(msg),
            qos,
        )

    end = time.time() + args.duration
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    for topic in args.topics:
        print(f"RESULT {topic}")
        for key, value in stats[topic].summary().items():
            print(f"  {key}: {value}")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
