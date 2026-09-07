from __future__ import annotations


def compact_list(items, *, limit: int = 12) -> list:
    values = list(items) if isinstance(items, (list, tuple)) else []
    return values[: max(int(limit), 0)]


def compact_quality_payload(quality: dict, *, limit: int = 12) -> dict:
    compact = dict(quality)
    for key in ("events", "anomalies"):
        if not isinstance(compact.get(key), list):
            continue
        compact[f"{key}_total"] = len(compact[key])
        compact[key] = compact_list(compact[key], limit=limit)
        compact[f"{key}_truncated"] = compact[f"{key}_total"] > len(compact[key])
    return compact


def compact_replay_payload(payload: dict, *, limit: int = 12) -> dict:
    compact = dict(payload)
    for key in (
        "quality",
        "before_quality",
        "after_quality",
        "raw_quality",
        "filtered_quality",
        "retimed_quality",
    ):
        if isinstance(compact.get(key), dict):
            compact[key] = compact_quality_payload(compact[key], limit=limit)
    if isinstance(compact.get("anomalies"), list):
        compact["anomalies_total"] = len(compact["anomalies"])
        compact["anomalies"] = compact_list(compact["anomalies"], limit=limit)
        compact["anomalies_truncated"] = compact["anomalies_total"] > len(
            compact["anomalies"]
        )
    if isinstance(compact.get("prepared_replay"), dict):
        compact["prepared_replay"] = compact_replay_payload(
            compact["prepared_replay"],
            limit=limit,
        )
    return compact
