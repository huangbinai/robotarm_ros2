from __future__ import annotations

from rebotarm_teach.teach_replay_parameters import (
    TEACH_REPLAY_PARAMETER_DEFAULTS,
    declare_teach_replay_parameters,
)


def test_declares_shared_defaults_with_caller_specific_speed_name() -> None:
    declared = []

    declare_teach_replay_parameters(
        lambda name, default: declared.append((name, default)),
        speed_parameter_name="replay_speed",
    )

    assert declared == [("replay_speed", 1.0), *TEACH_REPLAY_PARAMETER_DEFAULTS]


def test_mutable_defaults_are_not_shared_between_declarations() -> None:
    first = []
    second = []
    declare_teach_replay_parameters(
        lambda name, default: first.append((name, default)),
        speed_parameter_name="speed",
    )
    declare_teach_replay_parameters(
        lambda name, default: second.append((name, default)),
        speed_parameter_name="speed",
    )

    first_defaults = dict(first)
    second_defaults = dict(second)
    first_defaults["max_replay_velocity_rad_s_by_joint"][0] = 99.0

    assert second_defaults["max_replay_velocity_rad_s_by_joint"] == [
        3.0,
        3.0,
        3.0,
        1.8,
        1.8,
        1.8,
    ]
