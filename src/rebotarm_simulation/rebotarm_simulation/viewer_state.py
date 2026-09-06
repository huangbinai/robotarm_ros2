from __future__ import annotations

from dataclasses import dataclass, replace

from .mujoco_dashboard import DASHBOARD_PAGES, PLOT_PAGES
from .mujoco_jog import JOG_SPEED_LEVELS


@dataclass(frozen=True)
class ViewerControlState:
    """Immutable operator and presentation state for the passive viewer."""

    selected_joint: int = 0
    joint_targets: tuple[float, ...] = (0.0,) * 6
    joint_positions: tuple[float, ...] = (0.0,) * 6
    joint_velocities: tuple[float, ...] = (0.0,) * 6
    gripper_width: float = 0.09
    gripper_actual_width: float = 0.09
    max_contact_force: float = 0.0
    contact_count: int = 0
    requested_torques: tuple[float, ...] = (0.0,) * 6
    applied_torques: tuple[float, ...] = (0.0,) * 6
    saturated: tuple[bool, ...] = (False,) * 6
    watchdog_remaining_s: float = 0.0
    collision_visible: bool = False
    paused: bool = False
    joint_delta: int = 0
    gripper_delta: int = 0
    joint_jog_direction: int = 0
    gripper_jog_direction: int = 0
    jog_time_remaining: float = 0.0
    single_step: bool = False
    reset: bool = False
    home: bool = False
    mode: str | None = None
    active_mode: str = "hold"
    jog_speed_index: int = 1
    joint_jog_rate: float = 0.20
    gripper_jog_rate: float = 0.02
    stop_jog: bool = False
    interaction_mode: str = "joint"
    selected_cartesian_axis: int = 0
    cartesian_frame: str = "world"
    cartesian_accumulator_s: float = 0.0
    ik_status: str = "idle"
    ik_error: float = 0.0
    dashboard_page: str = "overview"
    plot_page: str = "off"
    help_visible: bool = False
    ee_position: tuple[float, ...] = (0.0, 0.0, 0.0)
    ee_rpy: tuple[float, ...] = (0.0, 0.0, 0.0)
    target_position: tuple[float, ...] = (0.0, 0.0, 0.0)
    target_rpy: tuple[float, ...] = (0.0, 0.0, 0.0)
    cartesian_target_reset: bool = False
    quit: bool = False


def reduce_key(state: ViewerControlState, key: str) -> ViewerControlState:
    """Reduce one decoded key into state without touching MuJoCo."""
    key = key.lower()
    if key == "m":
        modes = ("joint", "xyz", "rpy")
        mode = modes[(modes.index(state.interaction_mode) + 1) % len(modes)]
        return replace(
            state,
            interaction_mode=mode,
            joint_jog_direction=0,
            gripper_jog_direction=0,
            cartesian_accumulator_s=0.0,
        )
    if key == "x":
        if state.interaction_mode != "joint":
            return replace(
                state,
                selected_cartesian_axis=(state.selected_cartesian_axis + 1) % 3,
                joint_jog_direction=0,
                stop_jog=bool(state.joint_jog_direction),
            )
        return replace(
            state,
            selected_joint=(state.selected_joint + 1) % 6,
            joint_jog_direction=0,
            stop_jog=bool(state.joint_jog_direction),
        )
    if key == "z":
        if state.interaction_mode != "joint":
            return replace(
                state,
                selected_cartesian_axis=(state.selected_cartesian_axis - 1) % 3,
                joint_jog_direction=0,
                stop_jog=bool(state.joint_jog_direction),
            )
        return replace(
            state,
            selected_joint=(state.selected_joint - 1) % 6,
            joint_jog_direction=0,
            stop_jog=bool(state.joint_jog_direction),
        )
    if key == "j":
        return replace(state, joint_jog_direction=-1, gripper_jog_direction=0)
    if key == "k":
        return replace(state, joint_jog_direction=1, gripper_jog_direction=0)
    if key == "c":
        return replace(state, joint_jog_direction=0, gripper_jog_direction=-1)
    if key == "o":
        return replace(state, joint_jog_direction=0, gripper_jog_direction=1)
    if key == "-":
        return replace(state, jog_speed_index=max(0, state.jog_speed_index - 1))
    if key in ("=", "+"):
        return replace(
            state,
            jog_speed_index=min(len(JOG_SPEED_LEVELS) - 1, state.jog_speed_index + 1),
        )
    if key == "s":
        return replace(
            state,
            joint_jog_direction=0,
            gripper_jog_direction=0,
            jog_time_remaining=0.0,
            stop_jog=True,
        )
    if key == " ":
        return replace(state, paused=not state.paused, single_step=False)
    if key == "." and state.paused:
        return replace(state, single_step=True)
    if key == "r":
        return replace(state, reset=True)
    if key == "t":
        return replace(state, home=True)
    if key == "g":
        return replace(
            state, mode="gravity_comp", joint_jog_direction=0, gripper_jog_direction=0
        )
    if key == "h":
        return replace(
            state, mode="hold", joint_jog_direction=0, gripper_jog_direction=0
        )
    if key == "p":
        return replace(
            state, mode="position", joint_jog_direction=0, gripper_jog_direction=0
        )
    if key == "v":
        return replace(state, collision_visible=not state.collision_visible)
    if key == "f9":
        page = PLOT_PAGES[(PLOT_PAGES.index(state.plot_page) + 1) % len(PLOT_PAGES)]
        return replace(state, plot_page=page, help_visible=False)
    if key == "f8":
        return replace(
            state,
            cartesian_frame="tool" if state.cartesian_frame == "world" else "world",
            joint_jog_direction=0,
        )
    if key == "f6":
        page = DASHBOARD_PAGES[
            (DASHBOARD_PAGES.index(state.dashboard_page) + 1) % len(DASHBOARD_PAGES)
        ]
        return replace(state, dashboard_page=page, help_visible=False)
    if key == "f7":
        return replace(state, help_visible=not state.help_visible, plot_page="off")
    if key in ("q", "\x1b"):
        return replace(state, quit=True)
    return state
