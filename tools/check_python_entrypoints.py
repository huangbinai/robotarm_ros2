#!/usr/bin/env python3
"""Read installed ROS Python shebangs without importing or starting nodes."""
from pathlib import Path
import sys


def main():
    root = Path(__file__).resolve().parents[1]
    expected_dir = root / '.venv-ros/bin'
    failures = []
    checked = 0
    for path in sorted((root / 'install-venv').glob('rebotarm*/lib/rebotarm*/*')):
        if not path.is_file():
            continue
        with path.open('rb') as stream:
            first = stream.readline(512).decode('utf-8', errors='replace').strip()
        if not first.startswith('#!') or 'python' not in first.lower():
            continue
        checked += 1
        interpreter = first[2:]
        if Path(interpreter).parent != expected_dir or not Path(interpreter).is_file():
            failures.append(f'{path.relative_to(root)}: {first}')
    for failure in failures:
        print('WRONG PYTHON:', failure)
    print(f'{checked} Python entrypoints checked; {len(failures)} incorrect')
    if not checked:
        print('No installed project entrypoints found; run bash tools/build_workspace.bash')
    return 0 if checked and not failures else 1


if __name__ == '__main__':
    raise SystemExit(main())
