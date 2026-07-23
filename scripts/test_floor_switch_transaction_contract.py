#!/usr/bin/env python3
"""Offline tests for the persisted unified floor-switch transaction contract."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_cloud_bridge"))

from m20pro_cloud_bridge.floor_switch_transaction_contract import (  # noqa: E402
    advance_transaction,
    begin_transaction,
    commit_decision,
    mark_uncertain_transaction,
    next_map_epoch,
    recover_interrupted_transaction,
    recover_uncertain_transaction,
    request_admission,
    rollback_decision,
)


def check(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def context() -> dict:
    return {
        "task_id": "task-1",
        "source_floor": "F1",
        "target_floor": "F2",
        "source_map_id": "map-f1",
        "target_map_id": "map-f2",
        "route": {"id": "route-up"},
    }


def transaction() -> dict:
    result = begin_transaction(
        request={"request_id": "switch-1", "plan_id": "task-1:run-1"},
        context=context(),
        source_map_digest="source-digest",
        target_map_digest="target-digest",
        map_epoch=1,
        now_text="now",
        source_factory_identity={
            "resolved_path": "/var/opt/robot/data/maps/source-1",
            "content_digest": "a" * 64,
            "identity_mode": "path",
        },
    )
    check(result["ok"], "transaction starts")
    return result["transaction"]


def test_restart_recovery_and_admission() -> None:
    tx = transaction()
    recovered = recover_interrupted_transaction(tx, now_text="restart", now_unix_s=100.0)
    check(recovered["changed"], "active transaction is recovered")
    check(recovered["transaction"]["state"] == "UNCERTAIN", "restart is fail-closed")
    blocked = request_admission(recovered["transaction"], "switch-2")
    check(blocked["code"] == "floor_switch_recovery_required", "uncertain state blocks new switch")
    check(request_admission(None, "switch-2")["ok"], "fresh state admits a request")
    legacy_failed = {
        "request_id": "old",
        "state": "FAILED",
        "message": "legacy failure",
    }
    recovered_failed = recover_interrupted_transaction(legacy_failed, now_text="restart")
    check(recovered_failed["changed"], "legacy failed transaction is recovered fail-closed")
    check(recovered_failed["transaction"]["state"] == "UNCERTAIN", "legacy failure becomes uncertain")
    legacy_uncertain = {"request_id": "legacy", "state": "UNCERTAIN"}
    migrated = recover_interrupted_transaction(
        legacy_uncertain,
        now_text="restart",
        now_unix_s=101.0,
    )
    check(migrated["changed"], "legacy uncertain transaction receives a fresh recovery barrier")
    check(migrated["transaction"]["uncertain_at_unix"] == 101.0, "recovery barrier is persisted")


def test_transaction_requires_reserved_connector_identity() -> None:
    missing_plan = begin_transaction(
        request={"request_id": "switch-1"},
        context=context(),
        source_map_digest="source-digest",
        target_map_digest="target-digest",
        map_epoch=1,
        now_text="now",
    )
    check(missing_plan["code"] == "floor_switch_plan_id_missing", "plan identity is mandatory")
    invalid_epoch = begin_transaction(
        request={"request_id": "switch-1", "plan_id": "task-1:run-1"},
        context=context(),
        source_map_digest="source-digest",
        target_map_digest="target-digest",
        map_epoch=0,
        now_text="now",
    )
    check(invalid_epoch["code"] == "floor_switch_map_epoch_invalid", "epoch must be pre-reserved")


def test_commit_requires_all_evidence() -> None:
    tx = transaction()
    applying = advance_transaction(tx, "APPLYING", message="apply", now_text="t1")
    relocalizing = advance_transaction(applying["transaction"], "RELOCALIZING", message="reloc", now_text="t2")
    pending = commit_decision(
        relocalizing["transaction"],
        task_active=True,
        target_map_id="map-f2",
        observed_map_digest="target-digest",
        factory_active_confirmed=True,
        relocalization={"confirmed": True, "navigation_ready": False},
        navigation_readiness={"ready": False},
    )
    check(pending["code"] == "floor_switch_target_navigation_not_ready", "relocalization alone cannot commit")
    accepted = commit_decision(
        relocalizing["transaction"],
        task_active=True,
        target_map_id="map-f2",
        observed_map_digest="target-digest",
        factory_active_confirmed=True,
        relocalization={"confirmed": True, "navigation_ready": True},
        navigation_readiness={"ready": True},
        factory_active_identity={"content_digest": "b" * 64},
    )
    check(accepted["ok"], "complete target evidence commits")

    web_confirmation = commit_decision(
        relocalizing["transaction"],
        task_active=True,
        target_map_id="map-f2",
        observed_map_digest="target-digest",
        factory_active_confirmed=True,
        relocalization={"confirmed": True, "navigation_ready": True},
        navigation_readiness={"ready": True},
        factory_active_identity={
            "active_content_digest": "c" * 64,
            "expected_content_digest": "c" * 64,
        },
    )
    check(web_confirmation["ok"], "Web active/expected digest confirmation commits")

    mismatched_identity = commit_decision(
        relocalizing["transaction"],
        task_active=True,
        target_map_id="map-f2",
        observed_map_digest="target-digest",
        factory_active_confirmed=True,
        relocalization={"confirmed": True, "navigation_ready": True},
        navigation_readiness={"ready": True},
        factory_active_identity={
            "active_content_digest": "d" * 64,
            "expected_content_digest": "e" * 64,
        },
    )
    check(
        mismatched_identity["code"] == "floor_switch_factory_identity_mismatch",
        "mismatched active/expected digest is rejected",
    )


def test_rollback_requires_source_relocalization() -> None:
    tx = transaction()
    applying = advance_transaction(tx, "APPLYING", message="apply", now_text="t1")
    rolling = advance_transaction(applying["transaction"], "ROLLING_BACK", message="rollback", now_text="t2")
    incomplete = rollback_decision(
        rolling["transaction"],
        source_map_id="map-f1",
        observed_map_digest="source-digest",
        factory_active_confirmed=True,
        relocalization={"confirmed": True, "navigation_ready": False},
        navigation_readiness={"ready": False},
    )
    check(incomplete["code"] == "floor_switch_rollback_navigation_not_ready", "map restore alone is not rollback")
    complete = rollback_decision(
        rolling["transaction"],
        source_map_id="map-f1",
        observed_map_digest="source-digest",
        factory_active_confirmed=True,
        relocalization={"confirmed": True, "navigation_ready": True},
        navigation_readiness={"ready": True},
    )
    check(complete["ok"], "fully restored source navigation rolls back")


def test_uncertain_state_is_the_only_emergency_closeout() -> None:
    tx = transaction()
    applying = advance_transaction(tx, "APPLYING", message="apply", now_text="t1")
    uncertain = mark_uncertain_transaction(
        applying["transaction"],
        message="external map state cannot be proven",
        now_text="t2",
    )
    check(uncertain["ok"], "active transaction can be closed as uncertain")
    check(uncertain["transaction"]["state"] == "UNCERTAIN", "uncertain state is persisted")
    replay = mark_uncertain_transaction(
        uncertain["transaction"],
        message="same evidence",
        now_text="t3",
    )
    check(replay["ok"], "uncertain state remains updateable with later evidence")
    committed = advance_transaction(
        transaction(), "APPLYING", message="apply", now_text="t1"
    )
    relocalizing = advance_transaction(
        committed["transaction"], "RELOCALIZING", message="reloc", now_text="t2"
    )
    done = advance_transaction(
        relocalizing["transaction"], "COMMITTED", message="done", now_text="t3"
    )
    blocked = mark_uncertain_transaction(
        done["transaction"],
        message="late exception",
        now_text="t4",
    )
    check(not blocked["ok"], "late exception cannot rewrite a committed transaction")


def uncertain_transaction() -> dict:
    tx = transaction()
    applying = advance_transaction(tx, "APPLYING", message="apply", now_text="t1")
    uncertain = mark_uncertain_transaction(
        applying["transaction"],
        message="physical state unknown",
        now_text="t2",
        now_unix_s=100.0,
    )
    check(uncertain["ok"], "uncertain recovery fixture is valid")
    return uncertain["transaction"]


def recovery(
    tx: dict,
    *,
    observed_digest: str = "selected-digest",
    factory_confirmed: bool = True,
    localization_status: Optional[dict] = None,
    navigation_readiness: Optional[dict] = None,
    relocalization_time: float = 101.0,
) -> dict:
    return recover_uncertain_transaction(
        tx,
        map_id="map-confirmed-by-operator",
        expected_map_digest="selected-digest",
        observed_map_digest=observed_digest,
        factory_active_confirmed=factory_confirmed,
        localization_status=(
            localization_status
            if localization_status is not None
            else {"confirmed": True, "map_relocalization_required": None}
        ),
        navigation_readiness=(
            navigation_readiness
            if navigation_readiness is not None
            else {"ready": True, "scan": {"fresh_after_barrier": True}}
        ),
        relocalization_time=relocalization_time,
        now_text="recovered",
    )


def test_uncertain_recovery_requires_complete_new_evidence() -> None:
    tx = uncertain_transaction()
    stale = recovery(tx, relocalization_time=99.0)
    check(stale["code"] == "floor_switch_recovery_relocalization_stale", "old 2101 cannot recover")

    map_mismatch = recovery(tx, observed_digest="other-map")
    check(map_mismatch["code"] == "floor_switch_recovery_map_mismatch", "104 map digest is mandatory")

    factory_missing = recovery(tx, factory_confirmed=False)
    check(
        factory_missing["code"] == "floor_switch_recovery_factory_unconfirmed",
        "106 active identity is mandatory",
    )

    localization_locked = recovery(
        tx,
        localization_status={
            "confirmed": True,
            "map_relocalization_required": {"code": "map_changed"},
        },
    )
    check(
        localization_locked["code"] == "floor_switch_recovery_localization_unconfirmed",
        "map relocalization lock is mandatory",
    )

    nav_not_ready = recovery(tx, navigation_readiness={"ready": False, "scan": {"ready": False}})
    check(
        nav_not_ready["code"] == "floor_switch_recovery_navigation_not_ready",
        "Nav2 and obstacle chain are mandatory",
    )


def test_complete_uncertain_recovery_is_terminal_and_admits_next_request() -> None:
    tx = uncertain_transaction()
    recovered = recovery(tx)
    check(recovered["ok"], "complete recovery evidence is accepted")
    check(recovered["transaction"]["state"] == "RECOVERED", "recovery terminal is persisted")
    check(
        recovered["transaction"]["recovered_map_id"] == "map-confirmed-by-operator",
        "operator-confirmed map is recorded",
    )
    check(request_admission(recovered["transaction"], "switch-2")["ok"], "recovery admits a new transaction")
    immutable = mark_uncertain_transaction(
        recovered["transaction"],
        message="late callback",
        now_text="late",
    )
    check(not immutable["ok"], "late callback cannot rewrite RECOVERED")

    missing_barrier = dict(tx)
    missing_barrier.pop("uncertain_at_unix", None)
    rejected = recovery(missing_barrier)
    check(
        rejected["code"] == "floor_switch_recovery_barrier_missing",
        "unknown recovery barrier fails closed",
    )


def main() -> int:
    check(next_map_epoch({"floor_switch_map_epoch": 7}) == 8, "epoch increments")
    test_restart_recovery_and_admission()
    test_transaction_requires_reserved_connector_identity()
    test_commit_requires_all_evidence()
    test_rollback_requires_source_relocalization()
    test_uncertain_state_is_the_only_emergency_closeout()
    test_uncertain_recovery_requires_complete_new_evidence()
    test_complete_uncertain_recovery_is_terminal_and_admits_next_request()
    print("floor switch transaction contract tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
