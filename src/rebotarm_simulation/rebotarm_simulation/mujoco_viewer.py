from __future__ import annotations

import argparse
from contextlib import nullcontext
from dataclasses import dataclass, replace
import importlib
import json
import math
from queue import Empty, SimpleQueue
import sys
import threading
import time
from typing import Callable, Sequence
import warnings

from .mujoco_commands import dispatch_sim_command
from .mujoco_sim import ARM_JOINT_NAMES, RebotArmMujoco


HELP = (
    "Z/X select | J/K start joint jog | C/O start gripper jog | S stop + hold\n"
    "-/+ speed | G gravity | H capture + hold | P track target | V collision\n"
    "T home | Q quit\n"
    "Terminal: joints J1..J6 | joint NAME VALUE | gripper WIDTH | state"
)
JOG_SPEED_LEVELS = (
    ("PRECISION", 0.25),
    ("NORMAL", 1.0),
    ("FAST", 2.5),
)
MODE_DESCRIPTIONS = {
    "position": "tracks saved target",
    "hold": "captures and holds pose",
    "gravity_comp": "gravity only; no target tracking",
    "raw_torque": "diagnostic torque + watchdog",
}
_RETAINED_UNSAFE_VIEWERS = []


@dataclass(frozen=True)
class ViewerControlState:
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
    quit: bool = False


def reduce_key(state: ViewerControlState, key: str) -> ViewerControlState:
    key = key.lower()
    if key == "x":
        return replace(
            state,
            selected_joint=(state.selected_joint + 1) % 6,
            joint_jog_direction=0,
            stop_jog=bool(state.joint_jog_direction),
        )
    if key == "z":
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
    if key in ("q", "\x1b"):
        return replace(state, quit=True)
    return state


def _take_key_snapshot(events: SimpleQueue) -> tuple[int, ...]:
    snapshot = []
    for _ in range(events.qsize()):
        try:
            snapshot.append(events.get_nowait())
        except Empty:
            break
    return tuple(snapshot)


def _take_line_snapshot(events: SimpleQueue) -> tuple[str, ...]:
    snapshot = []
    for _ in range(events.qsize()):
        try:
            snapshot.append(events.get_nowait())
        except Empty:
            break
    return tuple(snapshot)


def start_command_reader(command_stream, command_events: SimpleQueue) -> threading.Thread:
    def read_lines() -> None:
        for line in command_stream:
            command_events.put(str(line).strip())

    thread = threading.Thread(target=read_lines, name="mujoco-viewer-command-input", daemon=True)
    thread.start()
    return thread


def drain_key_events(events: SimpleQueue, state: ViewerControlState) -> ViewerControlState:
    """Consume a finite FIFO snapshot, leaving new events for the next cycle."""
    for keycode in _take_key_snapshot(events):
        state = reduce_key(state, _decode_key(keycode))
    return state


def process_command_events(sim, events: SimpleQueue, state: ViewerControlState, status_stream) -> ViewerControlState:
    for line in _take_line_snapshot(events):
        if not line:
            continue
        try:
            result = dispatch_sim_command(sim, line, paused=state.paused)
        except (TypeError, ValueError) as exc:
            print(f"command error: {exc}", file=status_stream, flush=True)
            continue
        state = replace(state, paused=result.paused, quit=state.quit or result.should_quit)
        if result.value is not None:
            print(f"command result: {_command_value_text(result.value)}", file=status_stream, flush=True)
        state = _state_from_sim(
            sim,
            paused=state.paused,
            selected_joint=state.selected_joint,
            previous=state,
        )
        state = replace(state, quit=state.quit)
        if state.quit:
            break
    return state


def _command_value_text(value) -> str:
    try:
        return json.dumps(_jsonable(value), ensure_ascii=False, sort_keys=True)
    except TypeError:
        return repr(value)


def _jsonable(value):
    if hasattr(value, "__dataclass_fields__"):
        return {name: _jsonable(getattr(value, name)) for name in value.__dataclass_fields__}
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if hasattr(value, "__dict__"):
        return {key: _jsonable(item) for key, item in vars(value).items()}
    return value


def process_key_events(
    sim,
    events: SimpleQueue,
    state: ViewerControlState,
    joint_step: float,
    gripper_step: float,
    jog_hold_time: float = 0.18,
) -> ViewerControlState:
    """Execute one finite event snapshot sequentially on the simulation thread."""
    for keycode in _take_key_snapshot(events):
        state = reduce_key(state, _decode_key(keycode))
        state = apply_pending_commands(
            sim, state, joint_step, gripper_step, jog_hold_time=jog_hold_time
        )
        if state.quit:
            break
        if state.single_step:
            sim.step()
            state = replace(state, single_step=False)
    return state


def _state_from_sim(
    sim,
    *,
    paused: bool = False,
    selected_joint: int = 0,
    previous: ViewerControlState | None = None,
) -> ViewerControlState:
    state = sim.get_state()
    targets = tuple(float(value) for value in sim.control_targets[:6])
    positions = tuple(float(value) for value in state.joint_positions[:6])
    velocities = tuple(float(value) for value in state.joint_velocities[:6])
    contacts = _contact_summary(sim)
    control = _control_status(sim)
    return ViewerControlState(
        selected_joint=selected_joint,
        joint_targets=targets,
        joint_positions=positions,
        joint_velocities=velocities,
        gripper_width=float(state.gripper_width),
        gripper_actual_width=float(state.gripper_width),
        max_contact_force=contacts[0],
        contact_count=contacts[1],
        paused=paused,
        requested_torques=control[0],
        applied_torques=control[1],
        saturated=control[2],
        watchdog_remaining_s=control[3],
        active_mode=control[4],
        collision_visible=False if previous is None else previous.collision_visible,
        jog_speed_index=1 if previous is None else previous.jog_speed_index,
        joint_jog_rate=0.20 if previous is None else previous.joint_jog_rate,
        gripper_jog_rate=0.02 if previous is None else previous.gripper_jog_rate,
    )


def _control_status(sim) -> tuple[tuple[float, ...], tuple[float, ...], tuple[bool, ...], float, str]:
    if hasattr(sim, "get_control_status"):
        status = sim.get_control_status()
        remaining = status.watchdog_remaining_s
        return (
            tuple(float(value) for value in status.requested_torques[:6]),
            tuple(float(value) for value in status.applied_torques[:6]),
            tuple(bool(value) for value in status.saturated[:6]),
            0.0 if remaining is None else float(remaining),
            str(status.mode),
        )
    mode = str(getattr(sim, "control_mode", "hold"))
    return (0.0,) * 6, (0.0,) * 6, (False,) * 6, 0.0, mode


def _contact_summary(sim) -> tuple[float, int]:
    if not hasattr(sim, "get_contacts"):
        return 0.0, 0
    contacts = tuple(sim.get_contacts())
    if not contacts:
        return 0.0, 0
    return max(float(contact.force) for contact in contacts), len(contacts)


def _refresh_observed_state(sim, state: ViewerControlState) -> ViewerControlState:
    sim_state = sim.get_state()
    max_contact_force, contact_count = _contact_summary(sim)
    control = _control_status(sim)
    return replace(
        state,
        joint_positions=tuple(float(value) for value in sim_state.joint_positions[:6]),
        joint_velocities=tuple(float(value) for value in sim_state.joint_velocities[:6]),
        gripper_actual_width=float(sim_state.gripper_width),
        max_contact_force=max_contact_force,
        contact_count=contact_count,
        requested_torques=control[0],
        applied_torques=control[1],
        saturated=control[2],
        watchdog_remaining_s=control[3],
        active_mode=control[4],
    )


def _set_mode(sim, mode: str) -> str:
    setter = getattr(sim, "set_mode", None) or getattr(sim, "set_control_mode")
    return str(setter(mode))


def _command_positions(sim, targets):
    command = getattr(sim, "command_joint_positions", None) or getattr(
        sim, "set_joint_position_targets"
    )
    return command(targets)


def _command_gripper(sim, width: float) -> float:
    command = getattr(sim, "command_gripper_width", None) or getattr(sim, "set_gripper_width")
    return float(command(width))


def apply_pending_commands(
    sim,
    state: ViewerControlState,
    joint_step: float,
    gripper_step: float,
    *,
    jog_hold_time: float = 0.18,
) -> ViewerControlState:
    if state.reset or state.home:
        pending_joint_delta = state.joint_delta
        pending_gripper_delta = state.gripper_delta
        quit_requested = state.quit
        if state.home:
            sim.reset_home()
        else:
            sim.reset()
        _set_mode(sim, "hold")
        state = _state_from_sim(
            sim,
            paused=state.paused,
            selected_joint=state.selected_joint,
            previous=state,
        )
        state = replace(
            state,
            joint_delta=pending_joint_delta,
            gripper_delta=pending_gripper_delta,
            quit=quit_requested,
        )

    if state.mode is not None:
        active_mode = _set_mode(sim, state.mode)
    else:
        active_mode = getattr(sim, "control_mode", state.active_mode)

    targets = state.joint_targets
    if state.joint_delta:
        name = ARM_JOINT_NAMES[state.selected_joint]
        requested = targets[state.selected_joint] + state.joint_delta * joint_step
        targets = tuple(_command_positions(sim, {name: requested}))
        active_mode = getattr(sim, "control_mode", "position")

    width = state.gripper_width
    if state.gripper_delta:
        width = _command_gripper(sim, width + state.gripper_delta * gripper_step)

    if state.stop_jog:
        active_mode = _set_mode(sim, "hold")

    state = replace(
        state,
        joint_targets=targets,
        gripper_width=width,
        joint_delta=0,
        gripper_delta=0,
        jog_time_remaining=0.0,
        reset=False,
        home=False,
        mode=None,
        stop_jog=False,
        active_mode=str(active_mode),
    )
    return _refresh_observed_state(sim, state)


def apply_continuous_jog(
    sim,
    state: ViewerControlState,
    *,
    dt: float,
    joint_rate: float,
    gripper_rate: float,
) -> ViewerControlState:
    if not state.joint_jog_direction and not state.gripper_jog_direction:
        return state

    targets = state.joint_targets
    speed_scale = JOG_SPEED_LEVELS[state.jog_speed_index][1]
    if state.joint_jog_direction:
        name = ARM_JOINT_NAMES[state.selected_joint]
        requested = (
            targets[state.selected_joint]
            + state.joint_jog_direction * joint_rate * speed_scale * dt
        )
        targets = tuple(_command_positions(sim, {name: requested}))

    width = state.gripper_width
    if state.gripper_jog_direction:
        width = _command_gripper(
            sim, width + state.gripper_jog_direction * gripper_rate * speed_scale * dt
        )

    state = replace(
        state,
        joint_targets=targets,
        gripper_width=width,
        jog_time_remaining=0.0,
        active_mode=str(getattr(sim, "control_mode", state.active_mode)),
    )
    return _refresh_observed_state(sim, state)


def _run_state_text(state: ViewerControlState) -> str:
    return "PAUSED" if state.paused else "RUNNING"


def _jog_state_text(state: ViewerControlState) -> str:
    if state.joint_jog_direction:
        sign = "+" if state.joint_jog_direction > 0 else "-"
        return f"{ARM_JOINT_NAMES[state.selected_joint]} {sign} (S to stop)"
    if state.gripper_jog_direction:
        action = "opening" if state.gripper_jog_direction > 0 else "closing"
        return f"gripper {action} (S to stop)"
    return "stopped"


def system_panel_text(state: ViewerControlState) -> tuple[str, str]:
    mode = state.active_mode
    description = MODE_DESCRIPTIONS.get(mode, "unknown control mode")
    saturation_count = sum(state.saturated)
    speed_name, speed_scale = JOG_SPEED_LEVELS[state.jog_speed_index]
    left = "SYSTEM\nMODE\nBEHAVIOR\nMOTION\nSPEED\nCOLLISION\nCONTACTS\nMAX FORCE\nSATURATION"
    right = (
        f"{_run_state_text(state)}\n{mode.upper()}\n{description}\n"
        f"{_jog_state_text(state)}\n"
        f"{speed_name} J:{state.joint_jog_rate * speed_scale:.2f} "
        f"G:{state.gripper_jog_rate * speed_scale:.3f}\n"
        f"{'SHOWN' if state.collision_visible else 'HIDDEN'}\n"
        f"{state.contact_count}\n{state.max_contact_force:.2f} N\n"
        f"{saturation_count}/6"
    )
    if mode == "raw_torque":
        left += "\nWATCHDOG"
        right += f"\n{state.watchdog_remaining_s:.3f} s"
    return left, right


def joint_panel_text(state: ViewerControlState) -> tuple[str, str]:
    header = "    J      Q       Q*      ERR      DQ"
    torque_header = " TAU REQ/OUT"
    rows = []
    torque_rows = []
    for index, name in enumerate(ARM_JOINT_NAMES):
        marker = ">" if index == state.selected_joint else " "
        actual = state.joint_positions[index]
        target = state.joint_targets[index]
        error = target - actual
        rows.append(
            f"{marker} J{index + 1} {actual:+7.3f} {target:+7.3f} "
            f"{error:+7.3f} {state.joint_velocities[index]:+7.3f}"
        )
        saturation = " !" if state.saturated[index] else ""
        torque_rows.append(
            f"{state.requested_torques[index]:+6.2f}/"
            f"{state.applied_torques[index]:+6.2f}{saturation}"
        )
    return "\n".join((header, *rows)), "\n".join((torque_header, *torque_rows))


def gripper_panel_text(state: ViewerControlState) -> tuple[str, str]:
    return (
        "GRIPPER\nACTUAL WIDTH\nTARGET WIDTH",
        f"\n{state.gripper_actual_width:.3f} m\n{state.gripper_width:.3f} m",
    )


def controls_panel_text() -> tuple[str, str]:
    return (
        "Z / X\nJ / K\nC / O\n- / +\nS\nG\nH\nP\nV\nT / R\nQ",
        "select joint\nstart joint - / +\nclose / open gripper\nspeed down / up\nSTOP + hold\n"
        "gravity only\ncapture + hold\ntrack saved target\ncollision view\n"
        "home / reset\nquit",
    )


def overlay_text(state: ViewerControlState) -> str:
    """Return a plain-text snapshot for diagnostics and backwards compatibility."""
    system_left, system_right = system_panel_text(state)
    joint_left, joint_right = joint_panel_text(state)
    gripper_left, gripper_right = gripper_panel_text(state)
    controls_left, controls_right = controls_panel_text()
    return (
        f"{system_left}\n{system_right}\n\n"
        f"{joint_left}\n{joint_right}\n\n"
        f"{gripper_left}\n{gripper_right}\n\n"
        f"{controls_left}\n{controls_right}\n\n{HELP}"
    )


def configure_viewer_rendering(viewer, *, collision_visible: bool) -> None:
    option = getattr(viewer, "opt", None)
    groups = getattr(option, "geomgroup", None)
    if option is None or groups is None or len(groups) < 4:
        return
    lock = getattr(viewer, "lock", None)
    with lock() if lock is not None else nullcontext():
        groups[2] = 1
        groups[3] = int(collision_visible)
        # The stock Simulate UI handles the same key event after our callback.
        # Restore its visualization flags so T/H/P/Z/X/J/C/O/V cannot make the
        # robot transparent, hide textures/lights, or add debug overlays while
        # those keys are being used as robot controls.
        flags = getattr(option, "flags", None)
        if flags is not None:
            mujoco = importlib.import_module("mujoco")
            for index, (_name, default, _shortcut) in enumerate(mujoco.mjVISSTRING):
                flags[index] = int(default)


def update_viewer_overlay(viewer, state: ViewerControlState) -> None:
    setter = getattr(viewer, "set_texts", None)
    if setter is not None:
        mujoco = importlib.import_module("mujoco")
        setter(
            [
                (
                    mujoco.mjtFontScale.mjFONTSCALE_100,
                    mujoco.mjtGridPos.mjGRID_TOPLEFT,
                    *system_panel_text(state),
                ),
                (
                    mujoco.mjtFontScale.mjFONTSCALE_100,
                    mujoco.mjtGridPos.mjGRID_TOPRIGHT,
                    *joint_panel_text(state),
                ),
                (
                    mujoco.mjtFontScale.mjFONTSCALE_100,
                    mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                    *gripper_panel_text(state),
                ),
                (
                    mujoco.mjtFontScale.mjFONTSCALE_100,
                    mujoco.mjtGridPos.mjGRID_BOTTOMRIGHT,
                    *controls_panel_text(),
                ),
            ]
        )


def _positive_float(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise argparse.ArgumentTypeError("must be a positive finite number")
    return number


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Control reBotArm in the MuJoCo viewer")
    parser.add_argument("--model", default=None, help="MuJoCo scene XML path")
    parser.add_argument("--joint-step", type=_positive_float, default=0.01, help="joint jog in radians")
    parser.add_argument("--gripper-step", type=_positive_float, default=0.001, help="gripper jog in metres")
    parser.add_argument(
        "--joint-rate",
        type=_positive_float,
        default=0.20,
        help="normal-gear joint jog rate in rad/s",
    )
    parser.add_argument(
        "--gripper-rate",
        type=_positive_float,
        default=0.02,
        help="normal-gear gripper jog rate in m/s",
    )
    parser.add_argument(
        "--jog-hold-time",
        type=_positive_float,
        default=0.18,
        help="seconds to keep jogging after the latest key-repeat event",
    )
    parser.add_argument(
        "--duration",
        type=_positive_float,
        default=None,
        help="exit after this many seconds of simulated time",
    )
    parser.add_argument(
        "--no-command-input",
        action="store_true",
        help="disable terminal line commands while the viewer is running",
    )
    parser.add_argument(
        "--verbose-status",
        action="store_true",
        help="also print changing real-time state to the terminal (off by default)",
    )
    return parser


def _decode_key(keycode: int) -> str:
    if keycode == 256:
        return "\x1b"
    if keycode == 333:  # GLFW_KEY_KP_SUBTRACT
        return "-"
    if keycode == 334:  # GLFW_KEY_KP_ADD
        return "+"
    try:
        return chr(keycode)
    except (TypeError, ValueError):
        return ""


def _launch_passive_viewer(launch_passive: Callable, model, data, on_key):
    """Launch while hiding one harmless GLFW/Wayland capability warning."""
    with warnings.catch_warnings():
        warnings.filterwarnings(
            "ignore",
            message=r".*Wayland: The platform does not provide the window position.*",
            category=Warning,
        )
        return launch_passive(
            model,
            data,
            key_callback=on_key,
            show_left_ui=False,
            show_right_ui=False,
        )


def _close_viewer_then_sim(
    viewer,
    sim,
    model,
    data,
    *,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
    timeout: float = 5.0,
) -> None:
    """Release native state only after the passive viewer thread lets it go."""
    if viewer is None:
        sim.close()
        return
    try:
        viewer.close()
    except BaseException:
        model_is_released = False
        try:
            model_is_released = viewer.m is None
        except BaseException:
            # If the public lifecycle signal is itself unavailable, releasing
            # native state would be an unsafe guess.
            pass
        if model_is_released:
            sim.close()
        else:
            _RETAINED_UNSAFE_VIEWERS.append((viewer, sim, model, data))
        raise

    viewer_model = getattr(viewer, "m", None)
    if viewer_model is None:
        sim.close()
        return
    started = clock()
    while viewer_model is not None:
        if clock() - started >= timeout:
            # Releasing MjModel/MjData while the viewer still exposes its model
            # can crash the process. Retain the complete ownership graph and
            # report the failed teardown instead of risking use-after-free.
            _RETAINED_UNSAFE_VIEWERS.append((viewer, sim, model, data))
            raise TimeoutError("MuJoCo passive viewer did not finish closing")
        sleep(0.01)
        viewer_model = getattr(viewer, "m", None)
    sim.close()


def main(
    argv: Sequence[str] | None = None,
    *,
    sim_factory: Callable = RebotArmMujoco,
    launch_passive: Callable | None = None,
    sleep: Callable[[float], None] = time.sleep,
    clock: Callable[[], float] = time.monotonic,
    status_stream=None,
    command_stream=None,
) -> int:
    args = build_parser().parse_args(argv)
    if status_stream is None:
        status_stream = sys.stderr
    sim = sim_factory(args.model)
    viewer = None
    model = data = None
    try:
        sim.reset_home()
        _set_mode(sim, "hold")
        if launch_passive is None:
            launch_passive = importlib.import_module("mujoco.viewer").launch_passive

        state = replace(
            _state_from_sim(sim),
            joint_jog_rate=args.joint_rate,
            gripper_jog_rate=args.gripper_rate,
        )
        start_simulation_time = float(sim.get_state().simulation_time)
        events = SimpleQueue()
        command_events = SimpleQueue()
        if command_stream is None:
            command_stream = sys.stdin
            command_input_enabled = (
                not args.no_command_input
                and hasattr(command_stream, "isatty")
                and command_stream.isatty()
            )
        else:
            command_input_enabled = not args.no_command_input
        if command_input_enabled:
            command_thread = start_command_reader(command_stream, command_events)
            if command_stream is not sys.stdin and (
                not hasattr(command_stream, "isatty") or not command_stream.isatty()
            ):
                command_thread.join(timeout=0.05)
        previous_status = overlay_text(state)
        print(
            "reBotArm MuJoCo viewer ready: Home + Hold. "
            "Real-time state is shown in the viewer; press Q to quit.",
            file=status_stream,
            flush=True,
        )

        def on_key(keycode: int) -> None:
            events.put(keycode)

        model, data = sim._unsafe_viewer_handles()
        viewer = _launch_passive_viewer(launch_passive, model, data, on_key)
        configure_viewer_rendering(viewer, collision_visible=False)
        update_viewer_overlay(viewer, state)
        try:
            while viewer.is_running():
                state = process_command_events(sim, command_events, state, status_stream)
                if state.quit:
                    break
                state = process_key_events(
                    sim,
                    events,
                    state,
                    args.joint_step,
                    args.gripper_step,
                    args.jog_hold_time,
                )
                if state.quit:
                    break
                cycle_start = clock()
                if not state.paused or state.single_step:
                    state = apply_continuous_jog(
                        sim,
                        state,
                        dt=sim.timestep,
                        joint_rate=args.joint_rate,
                        gripper_rate=args.gripper_rate,
                    )
                    sim.step()
                    state = _refresh_observed_state(sim, replace(state, single_step=False))
                current_status = overlay_text(state)
                if args.verbose_status and current_status != previous_status:
                    print(current_status, file=status_stream, flush=True)
                    previous_status = current_status
                configure_viewer_rendering(
                    viewer, collision_visible=state.collision_visible
                )
                update_viewer_overlay(viewer, state)
                viewer.sync()
                elapsed = float(sim.get_state().simulation_time) - start_simulation_time
                if args.duration is not None and elapsed >= args.duration:
                    break
                sleep(max(0.0, sim.timestep - (clock() - cycle_start)))
            return 0
        except KeyboardInterrupt:
            return 130
    finally:
        _close_viewer_then_sim(
            viewer,
            sim,
            model,
            data,
            clock=clock,
            sleep=sleep,
        )


if __name__ == "__main__":
    raise SystemExit(main())
