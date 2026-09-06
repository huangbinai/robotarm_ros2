"""Compatibility imports; calibration logic is owned by rebotarm_calibration."""

from rebotarm_calibration.tcp_calibration import (  # noqa: F401
    Vector3,
    average_offsets,
    estimate_sample_offset,
    format_tcp_offset_yaml,
)
