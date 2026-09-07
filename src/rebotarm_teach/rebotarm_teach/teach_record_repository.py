from __future__ import annotations

import json
from pathlib import Path

from .teach_record_types import PreparedTeachReplay, TeachSample


def encode_teach_sample(sample: TeachSample) -> str:
    return json.dumps(
        {
            "stamp": sample.stamp,
            "joint_names": list(sample.joint_names),
            "positions": list(sample.positions),
            "velocities": list(sample.velocities),
            "efforts": list(sample.efforts),
            "motor_status": sample.motor_status,
            "arm_state": sample.arm_state,
        },
        separators=(",", ":"),
    )


def decode_teach_sample(payload: str) -> TeachSample:
    data = json.loads(payload)
    return TeachSample(
        stamp=float(data["stamp"]),
        joint_names=tuple(str(value) for value in data["joint_names"]),
        positions=tuple(float(value) for value in data["positions"]),
        velocities=tuple(float(value) for value in data.get("velocities", [])),
        efforts=tuple(float(value) for value in data.get("efforts", [])),
        motor_status={str(key): int(value) for key, value in data.get("motor_status", {}).items()},
        arm_state=str(data.get("arm_state", "")),
    )


def load_teach_samples(path: str | Path) -> list[TeachSample]:
    samples: list[TeachSample] = []
    with Path(path).open("r", encoding="utf-8-sig") as handle:
        for line in handle:
            line = line.strip()
            if line:
                samples.append(decode_teach_sample(line))
    return samples


def prepared_record_path(raw_path: str | Path) -> Path:
    path = Path(raw_path)
    if path.suffix:
        return path.with_name(f"{path.stem}.prepared{path.suffix}")
    return path.with_name(f"{path.name}.prepared.jsonl")


def write_prepared_teach_record(
    raw_path: str | Path,
    prepared: PreparedTeachReplay,
    *,
    output_path: str | Path | None = None,
) -> Path:
    target = Path(output_path) if output_path is not None else prepared_record_path(raw_path)
    target.parent.mkdir(parents=True, exist_ok=True)
    samples: list[TeachSample]
    if prepared.retimed_points:
        joint_names = prepared.samples[0].joint_names if prepared.samples else ()
        samples = [
            TeachSample(
                stamp=float(point.time_from_start),
                joint_names=joint_names,
                positions=point.positions,
                velocities=point.velocities,
                efforts=(),
                motor_status={},
                arm_state="PREPARED_REPLAY",
            )
            for point in prepared.retimed_points
        ]
    else:
        samples = [
            TeachSample(
                stamp=float(index) / max(float(prepared.resample_rate_hz), 1.0),
                joint_names=sample.joint_names,
                positions=sample.positions,
                velocities=sample.velocities,
                efforts=sample.efforts,
                motor_status=sample.motor_status,
                arm_state="PREPARED_REPLAY",
            )
            for index, sample in enumerate(prepared.samples)
        ]
    with target.open("w", encoding="utf-8") as handle:
        for sample in samples:
            handle.write(encode_teach_sample(sample) + "\n")
    return target
