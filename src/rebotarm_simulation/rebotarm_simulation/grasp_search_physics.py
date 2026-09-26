"""Score independent, reproducible MuJoCo bottle grasp trials."""

from __future__ import annotations

import math
from collections.abc import Sequence

from .mujoco_sim import ARM_JOINT_NAMES, RebotArmMujoco
from .offline_trajectory import play_path


def evaluate_grasp_paths(
    paths: Sequence,
    *,
    initial_arm_positions: Sequence[float],
    close_width_m: float = 0.02,
    close_sec: float = 1.5,
    hold_sec: float = 1.0,
) -> dict:
    """Each call starts a fresh model; paths are pregrasp, grasp, lift."""
    if len(paths) != 3:
        raise ValueError("expected pregrasp, grasp and lift paths")
    initial = tuple(float(value) for value in initial_arm_positions)
    if len(initial) != 6 or any(not math.isfinite(value) for value in initial):
        raise ValueError("initial arm positions must contain six finite values")
    if not math.isfinite(close_width_m) or not 0 <= close_width_m <= 0.09:
        raise ValueError("close_width_m must be within [0, 0.09]")
    if any(not math.isfinite(value) or value <= 0 for value in (close_sec, hold_sec)):
        raise ValueError("close and hold times must be positive")

    with RebotArmMujoco() as sim:
        sim.reset_joint_positions(initial)
        initial_bottle = sim.get_state().object_poses["bottle"]
        sim.set_gripper_width(0.09)
        phases = {}
        def record_contact(state, contacts, phase):
            relevant = [contact for contact in contacts if "bottle" in (contact.body1, contact.body2)]
            left = any("left_finger_link" in (item.body1, item.body2) for item in relevant)
            right = any("right_finger_link" in (item.body1, item.body2) for item in relevant)
            report["bilateral_contact_steps"] += bool(left and right)
            if phase in {"close", "lift", "hold"}:
                report[f"{phase}_bilateral_steps"] += bool(left and right)
            report["max_bottle_force_n"] = max(
                report["max_bottle_force_n"], max((item.force for item in relevant), default=0.0)
            )
            report["max_bottle_penetration_m"] = max(
                report["max_bottle_penetration_m"],
                max((item.penetration_depth for item in relevant), default=0.0),
            )
            report["max_bottle_lift_m"] = max(
                report["max_bottle_lift_m"], state.object_poses["bottle"][2] - initial_bottle[2]
            )

        report = {
            "bilateral_contact_steps": 0,
            "close_bilateral_steps": 0,
            "lift_bilateral_steps": 0,
            "hold_bilateral_steps": 0,
            "max_bottle_force_n": 0.0,
            "max_bottle_penetration_m": 0.0,
            "max_bottle_lift_m": 0.0,
        }
        for stage, trajectory in zip(("pregrasp", "grasp"), paths[:2]):
            phases[stage] = play_path(
                sim, trajectory.points, trajectory.joint_names,
                observe=lambda state, contacts: record_contact(state, contacts, stage),
            )
        sim.set_gripper_width(close_width_m)
        for _ in range(max(1, round(close_sec / sim.timestep))):
            state = sim.step()
            record_contact(state, sim.get_contacts(), "close")
        phases["lift"] = play_path(
            sim, paths[2].points, paths[2].joint_names,
            observe=lambda state, contacts: record_contact(state, contacts, "lift"),
        )
        for _ in range(max(1, round(hold_sec / sim.timestep))):
            state = sim.step()
            record_contact(state, sim.get_contacts(), "hold")
        final = sim.get_state()
        final_bottle = final.object_poses["bottle"]
        final_lift = final_bottle[2] - initial_bottle[2]
        lateral = math.dist(final_bottle[:2], initial_bottle[:2])
        retained = final_lift >= 0.02 and lateral <= 0.04
        report.update(
            phases=phases,
            final_bottle_lift_m=final_lift,
            lateral_bottle_displacement_m=lateral,
            final_gripper_width_m=final.gripper_width,
            stable_lift=bool(retained and report["hold_bilateral_steps"]),
        )
        report["score"] = (
            100.0 * report["stable_lift"]
            + 4.0 * bool(report["close_bilateral_steps"])
            + 4.0 * bool(report["lift_bilateral_steps"])
            + 4.0 * bool(report["hold_bilateral_steps"])
            + 100.0 * min(max(final_lift, 0.0), 0.10)
            - 15.0 * lateral
            - 100.0 * report["max_bottle_penetration_m"]
        )
        return report
