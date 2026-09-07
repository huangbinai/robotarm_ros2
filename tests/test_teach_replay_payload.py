from __future__ import annotations

from rebotarm_teach.teach_replay_payload import compact_replay_payload


def test_compacts_quality_and_top_level_anomalies_without_mutating_input() -> None:
    payload = {
        "quality": {
            "events": [1, 2, 3],
            "anomalies": ["a", "b", "c"],
        },
        "anomalies": ["x", "y", "z"],
    }

    compact = compact_replay_payload(payload, limit=2)

    assert compact["quality"]["events"] == [1, 2]
    assert compact["quality"]["events_total"] == 3
    assert compact["quality"]["events_truncated"] is True
    assert compact["quality"]["anomalies"] == ["a", "b"]
    assert compact["anomalies"] == ["x", "y"]
    assert compact["anomalies_total"] == 3
    assert payload["quality"]["events"] == [1, 2, 3]
    assert payload["anomalies"] == ["x", "y", "z"]


def test_compacts_nested_prepared_replay_and_handles_zero_limit() -> None:
    payload = {
        "prepared_replay": {
            "after_quality": {"events": [1], "anomalies": ["a"]},
        }
    }

    compact = compact_replay_payload(payload, limit=0)

    quality = compact["prepared_replay"]["after_quality"]
    assert quality["events"] == []
    assert quality["events_total"] == 1
    assert quality["events_truncated"] is True
    assert quality["anomalies"] == []


def test_non_list_fields_are_preserved() -> None:
    payload = {"quality": {"events": "not-a-list"}, "anomalies": None}

    assert compact_replay_payload(payload) == payload
