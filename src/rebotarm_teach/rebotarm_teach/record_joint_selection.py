"""Keep the configured recording joint contract independent of backend extras."""
import math


def select_record_joints(message, joint_names):
    names = list(message.name)
    wanted = tuple(joint_names)
    if not wanted or len(set(wanted)) != len(wanted) or len(set(names)) != len(names):
        raise ValueError("recording joint names must be nonempty and unique")
    if len(message.position) != len(names) or any(name not in names for name in wanted):
        raise ValueError("joint state is missing configured recording joints/positions")
    order = [names.index(name) for name in wanted]
    values = []
    for field in ("position", "velocity", "effort"):
        vector = list(getattr(message, field))
        if not vector and field != "position":
            values.append(())
            continue
        if len(vector) != len(names):
            raise ValueError(f"joint state {field} length differs from joint names")
        selected = tuple(float(vector[index]) for index in order)
        if not all(math.isfinite(value) for value in selected):
            raise ValueError(f"joint state {field} contains non-finite values")
        values.append(selected)
    return tuple(values)
