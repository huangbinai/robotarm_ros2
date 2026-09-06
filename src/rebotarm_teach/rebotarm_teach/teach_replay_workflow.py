from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

from .teach_recording import (
    PreparedTeachReplay,
    TeachSample,
    load_teach_samples,
    prepared_teach_replay_to_dict,
    teach_trajectory_preview_to_dict,
    write_prepared_teach_record,
)


@dataclass(frozen=True)
class PreparedReplayRecord:
    source_path: str
    source_samples: list[TeachSample]
    prepared: PreparedTeachReplay
    prepared_path: str
    prepared_samples: list[TeachSample]
    payload: dict[str, Any]


class TeachReplayWorkflow:
    """Own record preparation and preview assembly outside dashboard transport code."""

    def prepare_record(
        self,
        record_path: str | Path,
        *,
        prepare_samples: Callable[[list[TeachSample]], PreparedTeachReplay],
    ) -> PreparedReplayRecord:
        source_path = str(record_path)
        source_samples = load_teach_samples(source_path)
        if not source_samples:
            raise ValueError("record contains no samples")
        prepared = prepare_samples(source_samples)
        prepared_path = str(write_prepared_teach_record(source_path, prepared))
        return PreparedReplayRecord(
            source_path=source_path,
            source_samples=source_samples,
            prepared=prepared,
            prepared_path=prepared_path,
            prepared_samples=load_teach_samples(prepared_path),
            payload=prepared_teach_replay_to_dict(prepared),
        )

    def build_preview(
        self,
        record_path: str | Path,
        *,
        max_points: int,
        prepare_samples: Callable[[list[TeachSample]], PreparedTeachReplay],
        collision_precheck: Callable[[list[TeachSample]], dict],
        record_info: Callable[[str], dict],
    ) -> dict:
        record = self.prepare_record(record_path, prepare_samples=prepare_samples)
        payload = teach_trajectory_preview_to_dict(record.prepared_samples, max_points=max_points)
        payload.update(
            {
                "accepted": True,
                "curve_source": "prepared",
                "path": record.prepared_path,
                "raw_record_path": record.source_path,
                "prepared_record_path": record.prepared_path,
                "prepared_replay": record.payload,
                "collision_precheck": collision_precheck(record.prepared_samples),
                "info": record_info(record.source_path),
            }
        )
        return payload
