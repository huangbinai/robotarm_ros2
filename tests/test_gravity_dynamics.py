from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from rebotarmcontroller.gravity_dynamics import GravityDynamicsAdapter


class _Model:
    def __init__(self, events) -> None:
        self._events = events

    def createData(self):
        self._events.append(("create_data",))
        return "model-data"

    def getFrameId(self, frame):
        self._events.append(("frame_id", frame))
        return 23


def test_gravity_dynamics_builds_model_once_and_forwards_torque() -> None:
    events = []

    def load_model():
        events.append(("load_model",))
        return _Model(events)

    def compute_gravity(*, q):
        events.append(("gravity", q))
        return q + 1.0

    adapter = GravityDynamicsAdapter.create(
        load_robot_model=load_model,
        compute_generalized_gravity=compute_gravity,
        pinocchio=SimpleNamespace(),
        end_effector_frame="end_link",
    )
    positions = np.array([1.0, 2.0])

    torque = adapter.gravity_torque(positions)

    np.testing.assert_allclose(torque, [2.0, 3.0])
    assert events[:3] == [
        ("load_model",),
        ("create_data",),
        ("frame_id", "end_link"),
    ]
    assert events[3][0] == "gravity"
    assert events[3][1] is positions


def test_gravity_dynamics_computes_linear_and_angular_speed() -> None:
    events = []
    jacobian = np.eye(6)
    model = object()
    data = object()

    def compute_joint_jacobians(received_model, received_data, positions):
        events.append(("jacobians", received_model, received_data, positions))

    def update_frame_placements(received_model, received_data):
        events.append(("placements", received_model, received_data))

    def get_frame_jacobian(received_model, received_data, frame_id, reference):
        events.append(
            ("frame_jacobian", received_model, received_data, frame_id, reference)
        )
        return jacobian

    pinocchio = SimpleNamespace(
        ReferenceFrame=SimpleNamespace(WORLD="world"),
        computeJointJacobians=compute_joint_jacobians,
        updateFramePlacements=update_frame_placements,
        getFrameJacobian=get_frame_jacobian,
    )
    adapter = GravityDynamicsAdapter(
        model=model,
        data=data,
        end_effector_frame_id=23,
        compute_generalized_gravity=lambda **_kwargs: np.zeros(6),
        pinocchio=pinocchio,
    )
    positions = np.arange(6, dtype=np.float64)
    velocities = np.array([3.0, 4.0, 0.0, 0.0, 0.0, 12.0])

    linear_speed, angular_speed = adapter.end_effector_speeds(
        positions,
        velocities,
    )

    assert linear_speed == pytest.approx(5.0)
    assert angular_speed == pytest.approx(12.0)
    assert events == [
        ("jacobians", model, data, positions),
        ("placements", model, data),
        ("frame_jacobian", model, data, 23, "world"),
    ]
