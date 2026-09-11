#!/usr/bin/env python3
"""Export the six arm joints from a legacy recording into a NEW JSONL file.

Does not send ROS commands or overwrite the original/output. Use after sourcing
the workspace. Re-run trajectory checks on the exported record before replay.
"""
import argparse
from pathlib import Path
from types import SimpleNamespace

from rebotarm_teach.record_joint_selection import select_record_joints
from rebotarm_teach.teach_record_repository import load_teach_samples, encode_teach_sample
from rebotarm_teach.teach_record_types import TeachSample


def convert(source: Path, target: Path) -> int:
    if source.resolve() == target.resolve():
        raise ValueError('Output must differ from the original recording')
    names = tuple(f'joint{i}' for i in range(1, 7))
    samples = load_teach_samples(source)
    if not samples:
        raise ValueError('record contains no samples')
    converted = []
    for sample in samples:
        position, velocity, effort = select_record_joints(SimpleNamespace(
            name=sample.joint_names, position=sample.positions,
            velocity=sample.velocities, effort=sample.efforts,
        ), names)
        converted.append(TeachSample(
            stamp=sample.stamp, joint_names=names, positions=position,
            velocities=velocity, efforts=effort,
            motor_status={name: value for name, value in sample.motor_status.items() if name in names},
            arm_state=sample.arm_state,
        ))
    with target.open('x', encoding='utf-8') as stream:
        for sample in converted:
            stream.write(encode_teach_sample(sample) + '\n')
    return len(converted)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('target', type=Path)
    args = parser.parse_args()
    count = convert(args.source, args.target)
    print(f'Wrote {count} six-axis samples to {args.target}; original preserved')


if __name__ == '__main__':
    main()
