#!/usr/bin/env python3
"""Offline audit for 106 edge scan feasibility artifacts.

The audit intentionally checks that this experiment remains outside production
launch paths until the battery-safe tests and service trial pass.
"""

from __future__ import annotations

import argparse
import os
import re
import stat
import sys
from pathlib import Path


REQUIRED_FILES = {
    "DECISION_REPORT_CN.md",
    "README.md",
    "FEASIBILITY_REPORT.md",
    "NEXT_BATTERY_TEST_PLAN.md",
    "SERVICE_TRIAL.md",
    "PRODUCTION_MIGRATION_PLAN.md",
    "REAL_CHAIN_STATUS.md",
    "build_on_106.sh",
    "run_balanced_demo_on_106.sh",
    "compare_scan_topics.py",
    "analyze_scan_bag.py",
    "drdds_lidar_probe.cpp",
    "drdds_edge_scan_demo.cpp",
    "service/m20pro-edge-scan-106.env.example",
    "service/m20pro-edge-scan-106.env.edge_scan",
    "service/m20pro-edge-scan-106.service.example",
}

EXECUTABLE_FILES = {
    "build_on_106.sh",
    "run_balanced_demo_on_106.sh",
    "compare_scan_topics.py",
    "analyze_scan_bag.py",
}

FORBIDDEN_PRODUCTION_PATTERNS = (
    "scan_edge_exp",
)

PRODUCTION_DIRS = (
    "src",
    "scripts",
    "systemd",
)

ALLOWED_TEXT_ROOTS = (
    Path("tools/edge_scan_feasibility"),
)


def fail(message: str) -> None:
    print(f"FAIL: {message}")
    raise SystemExit(1)


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace")


def assert_required_files(root: Path) -> None:
    missing = [name for name in sorted(REQUIRED_FILES) if not (root / name).is_file()]
    if missing:
        fail("missing required files: " + ", ".join(missing))


def assert_executable_bits(root: Path) -> None:
    wrong = []
    for name in sorted(EXECUTABLE_FILES):
        path = root / name
        mode = path.stat().st_mode
        if not (mode & stat.S_IXUSR):
            wrong.append(name)
    if wrong:
        fail("files should be executable: " + ", ".join(wrong))


def assert_demo_defaults(root: Path) -> None:
    runner = read_text(root / "run_balanced_demo_on_106.sh")
    expected = {
        "OUTPUT_TOPIC": "/m20pro/scan_edge_exp",
        "DURATION_S": "90",
        "HEIGHT_MIN": "-0.05",
        "HEIGHT_MAX": "0.55",
        "MAX_PUBLISH_HZ": "4",
        "MAX_POINTS": "12000",
        "FRAME_ID": "m20pro_base_link",
        "ANGLE_INCREMENT": "0.0174533",
        "RANGE_MAX": "10.0",
        "RANGE_MIN": "0.2",
    }
    for key, value in expected.items():
        pattern = re.compile(rf'{key}="\$\{{{key}:-{re.escape(value)}\}}"')
        if not pattern.search(runner):
            fail(f"balanced runner default mismatch: {key} should default to {value}")

    env = read_text(root / "service/m20pro-edge-scan-106.env.example")
    if "DURATION_S=0" not in env:
        fail("service env example must use DURATION_S=0 for manual service trial")
    if "OUTPUT_TOPIC=/m20pro/scan_edge_exp" not in env:
        fail("service env example must publish the experimental topic")
    edge_env = read_text(root / "service/m20pro-edge-scan-106.env.edge_scan")
    if "OUTPUT_TOPIC=/scan" not in edge_env:
        fail("edge scan env must publish /scan for the real edge_scan chain")
    if "DURATION_S=0" not in edge_env:
        fail("edge scan env must run until stopped")


def assert_duration_zero_supported(root: Path) -> None:
    source = read_text(root / "drdds_edge_scan_demo.cpp")
    if "cfg.duration_s <= 0" not in source:
        fail("drdds_edge_scan_demo.cpp must support duration_s<=0 for service trial")


def is_binary(path: Path) -> bool:
    try:
        with path.open("rb") as handle:
            return b"\0" in handle.read(4096)
    except OSError:
        return True


def iter_text_files(paths: list[Path]):
    for base in paths:
        if not base.exists():
            continue
        for path in base.rglob("*"):
            if path.is_file() and not is_binary(path):
                yield path


def assert_not_wired_into_production(repo: Path) -> None:
    offenders: list[str] = []
    bases = [repo / name for name in PRODUCTION_DIRS]
    for path in iter_text_files(bases):
        text = read_text(path)
        for pattern in FORBIDDEN_PRODUCTION_PATTERNS:
            if pattern in text:
                offenders.append(f"{path.relative_to(repo)} contains {pattern}")
    if offenders:
        fail("edge scan experiment appears in production paths: " + "; ".join(offenders))


def assert_docs_reference_plans(root: Path) -> None:
    readme = read_text(root / "README.md")
    report = read_text(root / "FEASIBILITY_REPORT.md")
    for required in (
        "NEXT_BATTERY_TEST_PLAN.md",
        "SERVICE_TRIAL.md",
        "PRODUCTION_MIGRATION_PLAN.md",
        "REAL_CHAIN_STATUS.md",
        "analyze_scan_bag.py",
    ):
        if required not in readme:
            fail(f"README.md does not reference {required}")
        if required not in report:
            fail(f"FEASIBILITY_REPORT.md does not reference {required}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--repo",
        default=str(Path(__file__).resolve().parents[2]),
        help="repository root",
    )
    args = parser.parse_args()
    repo = Path(args.repo).resolve()
    root = repo / "tools" / "edge_scan_feasibility"

    if not root.is_dir():
        fail(f"missing edge scan directory: {root}")

    assert_required_files(root)
    assert_executable_bits(root)
    assert_demo_defaults(root)
    assert_duration_zero_supported(root)
    assert_not_wired_into_production(repo)
    assert_docs_reference_plans(root)

    print("OK: edge scan artifacts are complete; experimental topic is isolated and edge_scan switch files are present")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
