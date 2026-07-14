"""Pure helpers for querying radar result records."""

from typing import Any, Dict, Iterable, List


def _scalar_texts(value: Any, depth: int = 0) -> Iterable[str]:
    if depth > 6:
        return
    if isinstance(value, dict):
        for key, item in value.items():
            yield str(key)
            yield from _scalar_texts(item, depth + 1)
        return
    if isinstance(value, (list, tuple)):
        for item in value:
            yield from _scalar_texts(item, depth + 1)
        return
    if value is not None:
        yield str(value)


def radar_job_search_values(job: Dict[str, Any]) -> List[str]:
    """Return searchable public metadata without indexing raw device payloads."""
    waypoint = job.get("waypoint") if isinstance(job.get("waypoint"), dict) else {}
    request = job.get("request") if isinstance(job.get("request"), dict) else {}
    summary = job.get("summary") if isinstance(job.get("summary"), dict) else {}
    manual = job.get("manual_measurement") if isinstance(job.get("manual_measurement"), dict) else {}
    direct = {
        key: job.get(key)
        for key in (
            "task_id",
            "taskId",
            "run_id",
            "waypoint_key",
            "scan_mode",
            "scan_label",
            "status",
            "state",
            "started_at",
            "finished_at",
        )
    }
    return list(_scalar_texts((direct, waypoint, request, summary, manual)))


def radar_job_matches_query(job: Dict[str, Any], query: Any) -> bool:
    tokens = [token.casefold() for token in str(query or "").split() if token.strip()]
    if not tokens:
        return True
    haystack = "\n".join(radar_job_search_values(job)).casefold()
    return all(token in haystack for token in tokens)
