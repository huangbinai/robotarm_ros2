from __future__ import annotations

from rebotarm_teach import teach_recording
from rebotarm_teach.teach_record_repository import (
    decode_teach_sample,
    encode_teach_sample,
    load_teach_samples,
    prepared_record_path,
)
from rebotarm_teach.teach_record_types import TeachSample


def test_teach_recording_keeps_compatibility_exports() -> None:
    assert teach_recording.TeachSample is TeachSample
    assert teach_recording.encode_teach_sample is encode_teach_sample
    assert teach_recording.decode_teach_sample is decode_teach_sample
    assert teach_recording.load_teach_samples is load_teach_samples
    assert teach_recording.prepared_record_path is prepared_record_path


def test_jsonl_repository_round_trips_utf8_samples(tmp_path) -> None:
    sample = TeachSample(
        stamp=1.25,
        joint_names=("joint1", "joint2"),
        positions=(0.1, -0.2),
        velocities=(0.3, -0.4),
        efforts=(0.5, 0.6),
        motor_status={"joint1": 0},
        arm_state="示教",
    )
    path = tmp_path / "record.jsonl"
    path.write_text(encode_teach_sample(sample) + "\n", encoding="utf-8")

    loaded = load_teach_samples(path)

    assert loaded == [sample]
    assert prepared_record_path(path) == tmp_path / "record.prepared.jsonl"
