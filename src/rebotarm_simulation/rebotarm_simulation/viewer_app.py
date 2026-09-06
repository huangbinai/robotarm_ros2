from __future__ import annotations

from dataclasses import dataclass, replace
import importlib
from queue import SimpleQueue
import sys
from typing import Callable
import warnings

from .mujoco_cartesian import MujocoCartesianController
from .mujoco_telemetry import MujocoTelemetryHistory
from .mujoco_visualization import GhostArmOverlay, TelemetryFigures
from .viewer_input import start_command_reader
from .viewer_runtime import close_viewer_then_sim
from .viewer_state import ViewerControlState


TELEMETRY_SAMPLE_HZ = 50.0
TELEMETRY_WINDOW_S = 10.0
VISUAL_UPDATE_HZ = 30.0
DASHBOARD_UPDATE_HZ = 10.0


@dataclass(frozen=True)
class ViewerAppActions:
    """Operations supplied by the control and presentation layers."""

    state_from_sim: Callable
    set_mode: Callable
    process_command_events: Callable
    process_key_events: Callable
    refresh_observed_state: Callable
    apply_continuous_jog: Callable
    overlay_text: Callable
    configure_rendering: Callable
    update_overlay: Callable
    update_ghost: Callable
    clear_ghost: Callable
    align_cartesian_target: Callable
    process_cartesian_target: Callable
    quaternion_wxyz_to_rpy: Callable


def launch_passive_viewer(launch_passive: Callable, model, data, on_key):
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


def run_viewer_app(
    sim,
    args,
    actions: ViewerAppActions,
    *,
    launch_passive: Callable | None,
    sleep: Callable[[float], None],
    clock: Callable[[], float],
    status_stream,
    command_stream,
) -> int:
    """Own the passive-viewer lifecycle and frame scheduling."""
    viewer = None
    model = data = None
    try:
        sim.reset_home()
        actions.set_mode(sim, "hold")
        if launch_passive is None:
            launch_passive = importlib.import_module("mujoco.viewer").launch_passive

        state = replace(
            actions.state_from_sim(sim),
            joint_jog_rate=args.joint_rate,
            gripper_jog_rate=args.gripper_rate,
        )
        cartesian_controller = (
            MujocoCartesianController(sim)
            if hasattr(sim, "kinematics_adapter")
            else None
        )
        telemetry = MujocoTelemetryHistory(
            capacity=round(TELEMETRY_SAMPLE_HZ * TELEMETRY_WINDOW_S)
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
        previous_status = actions.overlay_text(state)
        print(
            "reBotArm MuJoCo viewer ready: Home + Hold. "
            "Real-time state is shown in the viewer; press Q to quit.",
            file=status_stream,
            flush=True,
        )

        def on_key(keycode: int) -> None:
            events.put(keycode)

        model, data = sim.render_adapter.handles()
        ghost = GhostArmOverlay(model)
        figures = TelemetryFigures(max_points=500)
        target_mocap_id = None
        target_pose = None
        if cartesian_controller is not None:
            target_mocap_id, target_position, target_quaternion = (
                actions.align_cartesian_target(model, data)
            )
            target_pose = (target_position, target_quaternion)
            state = replace(
                state,
                target_position=target_position,
                target_rpy=actions.quaternion_wxyz_to_rpy(target_quaternion),
            )
        viewer = launch_passive_viewer(launch_passive, model, data, on_key)
        actions.configure_rendering(
            viewer, collision_visible=False, target_visible=False
        )
        actions.update_overlay(viewer, state)
        displayed_dashboard = actions.overlay_text(state)
        last_visual_update_sim_time = float("-inf")
        last_telemetry_sample_sim_time = float("-inf")
        last_dashboard_update_sim_time = float("-inf")
        plots_attached = False
        ghost_visible = False
        try:
            while viewer.is_running():
                state = actions.process_command_events(
                    sim, command_events, state, status_stream
                )
                if state.quit:
                    break
                state = actions.process_key_events(
                    sim,
                    events,
                    state,
                    args.joint_step,
                    args.gripper_step,
                    args.jog_hold_time,
                )
                if state.quit:
                    break
                if cartesian_controller is not None and state.cartesian_target_reset:
                    target_mocap_id, target_position, target_quaternion = (
                        actions.align_cartesian_target(model, data)
                    )
                    target_pose = (target_position, target_quaternion)
                    state = replace(
                        state,
                        cartesian_target_reset=False,
                        ik_status="idle",
                        target_position=target_position,
                        target_rpy=actions.quaternion_wxyz_to_rpy(target_quaternion),
                    )
                if cartesian_controller is not None and target_pose is not None:
                    target_pose, state = actions.process_cartesian_target(
                        cartesian_controller,
                        data,
                        target_mocap_id,
                        target_pose,
                        state,
                    )
                cycle_start = clock()
                if not state.paused or state.single_step:
                    state = actions.apply_continuous_jog(
                        sim,
                        state,
                        dt=sim.timestep,
                        joint_rate=args.joint_rate,
                        gripper_rate=args.gripper_rate,
                        cartesian_controller=cartesian_controller,
                    )
                    sim.step()
                    state = actions.refresh_observed_state(
                        sim, replace(state, single_step=False)
                    )
                    simulation_time = float(sim.get_state().simulation_time)
                    telemetry_due = (
                        simulation_time < last_telemetry_sample_sim_time
                        or simulation_time - last_telemetry_sample_sim_time
                        >= 1.0 / TELEMETRY_SAMPLE_HZ
                    )
                    if telemetry_due and hasattr(sim, "get_control_status"):
                        try:
                            telemetry.append(
                                float(sim.get_state().simulation_time),
                                sim.get_control_status(),
                                sim.get_contacts() if hasattr(sim, "get_contacts") else (),
                            )
                        except (AttributeError, TypeError, ValueError):
                            pass
                        last_telemetry_sample_sim_time = simulation_time
                current_status = actions.overlay_text(state)
                if args.verbose_status and current_status != previous_status:
                    print(current_status, file=status_stream, flush=True)
                    previous_status = current_status
                actions.configure_rendering(
                    viewer,
                    collision_visible=state.collision_visible,
                    target_visible=state.interaction_mode != "joint",
                )
                simulation_time = float(sim.get_state().simulation_time)
                visual_update_due = (
                    simulation_time < last_visual_update_sim_time
                    or simulation_time - last_visual_update_sim_time
                    >= 1.0 / VISUAL_UPDATE_HZ
                )
                if visual_update_due:
                    if state.interaction_mode != "joint":
                        ghost_visible = actions.update_ghost(
                            ghost, viewer, state.joint_targets
                        )
                    elif ghost_visible:
                        actions.clear_ghost(ghost, viewer)
                        ghost_visible = False
                    if state.plot_page != "off":
                        figures.select(
                            "tracking" if state.plot_page == "tracking" else "torque"
                        )
                        figures.update(
                            telemetry.snapshot(), joint_index=state.selected_joint
                        )
                        plots_attached = figures.attach_active(viewer)
                    last_visual_update_sim_time = simulation_time
                if state.plot_page == "off" and plots_attached:
                    figures.clear(viewer)
                    plots_attached = False
                dashboard_due = (
                    simulation_time < last_dashboard_update_sim_time
                    or simulation_time - last_dashboard_update_sim_time
                    >= 1.0 / DASHBOARD_UPDATE_HZ
                    or current_status != displayed_dashboard
                )
                if dashboard_due:
                    actions.update_overlay(viewer, state)
                    displayed_dashboard = current_status
                    last_dashboard_update_sim_time = simulation_time
                viewer.sync()
                elapsed = float(sim.get_state().simulation_time) - start_simulation_time
                if args.duration is not None and elapsed >= args.duration:
                    break
                sleep(max(0.0, sim.timestep - (clock() - cycle_start)))
            return 0
        except KeyboardInterrupt:
            return 130
    finally:
        close_viewer_then_sim(
            viewer,
            sim,
            model,
            data,
            clock=clock,
            sleep=sleep,
        )
