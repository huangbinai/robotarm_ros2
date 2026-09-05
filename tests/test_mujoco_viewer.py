from __future__ import annotations

from dataclasses import replace
import io
from queue import SimpleQueue
import threading
from types import SimpleNamespace
from collections.abc import Mapping
import warnings

import pytest

from rebotarm_simulation import mujoco_viewer
from rebotarm_simulation.mujoco_sim import RebotArmMujoco


class FakeSim:
    joint_names = tuple(f"joint{i}" for i in range(1, 7)) + (
        "left_finger_joint",
        "right_finger_joint",
    )

    def __init__(self, model_path=None):
        self.model_path = model_path
        self.viewer_model = object()
        self.viewer_data = object()
        self.timestep = 0.01
        self.targets = [0.0] * 6
        self.width = 0.05
        self.time = 0.0
        self.control_mode = "hold"
        self.calls = []
        self.call_threads = []
        self.closed = False

    @property
    def control_targets(self):
        return tuple(self.targets) + (self.width / 2.0, -self.width / 2.0)

    def _unsafe_viewer_handles(self):
        return self.viewer_model, self.viewer_data

    def get_state(self):
        return SimpleNamespace(
            joint_positions=tuple(self.targets) + (self.width / 2.0, -self.width / 2.0),
            joint_velocities=(0.01,) * 6 + (0.0, 0.0),
            gripper_width=self.width,
            simulation_time=self.time,
        )

    def get_contacts(self):
        return (SimpleNamespace(force=2.5), SimpleNamespace(force=0.4))

    def set_joint_position_targets(self, targets):
        self.call_threads.append(threading.get_ident())
        self.calls.append(("joints", dict(targets) if isinstance(targets, Mapping) else list(targets)))
        if isinstance(targets, Mapping):
            for name, value in targets.items():
                index = int(name.removeprefix("joint")) - 1
                self.targets[index] = min(1.0, max(-1.0, float(value)))
        else:
            self.targets[:] = [min(1.0, max(-1.0, float(value))) for value in targets]
        return tuple(self.targets)

    def set_gripper_width(self, width):
        self.call_threads.append(threading.get_ident())
        self.calls.append(("gripper", width))
        self.width = min(0.09, max(0.0, float(width)))
        return self.width

    def set_control_mode(self, mode):
        self.calls.append(("mode", mode))
        self.control_mode = mode
        return mode

    set_mode = set_control_mode
    command_joint_positions = set_joint_position_targets
    command_gripper_width = set_gripper_width

    def get_control_status(self):
        return SimpleNamespace(
            mode=self.control_mode,
            requested_torques=(0.0,) * 6,
            applied_torques=(0.0,) * 6,
            saturated=(False,) * 6,
            watchdog_remaining_s=0.0,
        )

    def step(self):
        self.call_threads.append(threading.get_ident())
        self.calls.append(("step",))
        self.time += self.timestep
        return self.get_state()

    def reset(self):
        self.calls.append(("reset",))
        self.targets[:] = [0.0] * 6
        self.width = 0.09
        self.time = 0.0
        return self.get_state()

    def reset_home(self):
        self.calls.append(("reset_home",))
        self.targets[:] = [0.0] * 6
        self.width = 0.09
        self.time = 0.0
        return self.get_state()

    def close(self):
        self.calls.append(("close",))
        self.closed = True


class FakeViewer:
    def __init__(self, callback, keys):
        self.callback = callback
        self.keys = iter(keys)
        self.sync_count = 0
        self.closed = False
        self.opt = SimpleNamespace(geomgroup=[1] * 6)
        self.texts = None

    def is_running(self):
        try:
            self.callback(next(self.keys))
            return True
        except StopIteration:
            return False

    def sync(self):
        self.sync_count += 1

    def close(self):
        self.closed = True

    def set_texts(self, texts):
        self.texts = texts


def test_internal_viewer_handles_reject_closed_simulation():
    sim = RebotArmMujoco.__new__(RebotArmMujoco)
    sim._closed = True
    with pytest.raises(RuntimeError, match="closed"):
        sim._unsafe_viewer_handles()


def test_control_state_selects_six_arm_joints_with_wrapping():
    state = mujoco_viewer.ViewerControlState()
    state = mujoco_viewer.reduce_key(state, "x")
    assert state.selected_joint == 1
    state = mujoco_viewer.reduce_key(state, "z")
    state = mujoco_viewer.reduce_key(state, "z")
    assert state.selected_joint == 5
    assert mujoco_viewer.reduce_key(state, "3").selected_joint == 5


def test_project_rendering_restores_stock_shortcut_flags_and_geom_groups():
    mujoco = pytest.importorskip("mujoco")
    option = mujoco.MjvOption()
    option.geomgroup[2] = 0
    option.geomgroup[3] = 1
    transparent = int(mujoco.mjtVisFlag.mjVIS_TRANSPARENT)
    texture = int(mujoco.mjtVisFlag.mjVIS_TEXTURE)
    option.flags[transparent] = 1
    option.flags[texture] = 0
    viewer = SimpleNamespace(opt=option)

    mujoco_viewer.configure_viewer_rendering(viewer, collision_visible=False)

    assert option.geomgroup[2] == 1
    assert option.geomgroup[3] == 0
    assert option.flags[transparent] == 0
    assert option.flags[texture] == 1


@pytest.mark.parametrize(
    ("key", "field", "value"),
    [
        ("j", "joint_jog_direction", -1),
        ("k", "joint_jog_direction", 1),
        ("c", "gripper_jog_direction", -1),
        ("o", "gripper_jog_direction", 1),
        (".", "single_step", True),
        ("r", "reset", True),
        ("q", "quit", True),
    ],
)
def test_key_reducer_maps_commands(key, field, value):
    result = mujoco_viewer.reduce_key(mujoco_viewer.ViewerControlState(paused=True), key)
    assert getattr(result, field) == value


def test_reducer_uses_latched_single_axis_jog_commands():
    state = mujoco_viewer.ViewerControlState()
    for key in ("k", "k", "j", "o", "o", "c"):
        state = mujoco_viewer.reduce_key(state, key)
    assert state.joint_jog_direction == 0
    assert state.gripper_jog_direction == -1


def test_stop_key_cancels_all_jog_and_requests_hold():
    state = mujoco_viewer.ViewerControlState(
        joint_jog_direction=1,
        gripper_jog_direction=-1,
    )
    state = mujoco_viewer.reduce_key(state, "s")
    assert state.joint_jog_direction == 0
    assert state.gripper_jog_direction == 0
    assert state.stop_jog is True


def test_speed_keys_select_bounded_precision_normal_fast_and_turbo_gears():
    state = mujoco_viewer.ViewerControlState()
    assert mujoco_viewer.JOG_SPEED_LEVELS[state.jog_speed_index][0] == "NORMAL"

    state = mujoco_viewer.reduce_key(state, "-")
    assert mujoco_viewer.JOG_SPEED_LEVELS[state.jog_speed_index][0] == "PRECISION"
    assert mujoco_viewer.reduce_key(state, "-").jog_speed_index == 0

    state = mujoco_viewer.reduce_key(state, "+")
    state = mujoco_viewer.reduce_key(state, "=")
    assert mujoco_viewer.JOG_SPEED_LEVELS[state.jog_speed_index][0] == "FAST"
    state = mujoco_viewer.reduce_key(state, "+")
    assert mujoco_viewer.JOG_SPEED_LEVELS[state.jog_speed_index][0] == "TURBO"
    assert mujoco_viewer.reduce_key(state, "+").jog_speed_index == 3


def test_keypad_plus_and_minus_control_speed_gears():
    state = mujoco_viewer.ViewerControlState()
    state = mujoco_viewer.reduce_key(state, mujoco_viewer._decode_key(334))
    assert state.jog_speed_index == 2
    state = mujoco_viewer.reduce_key(state, mujoco_viewer._decode_key(333))
    assert state.jog_speed_index == 1


def test_drain_key_events_preserves_burst_order_and_counts():
    events = SimpleQueue()
    for key in (ord("k"), ord("k"), ord("j"), ord("o"), ord(" "), ord(".")):
        events.put(key)
    state = mujoco_viewer.drain_key_events(events, mujoco_viewer.ViewerControlState())
    assert state.joint_jog_direction == 0
    assert state.gripper_jog_direction == 1
    assert state.paused is True
    assert state.single_step is True
    assert events.empty()


def test_drain_processes_finite_snapshot_when_producer_keeps_adding():
    class GrowingQueue:
        def __init__(self):
            self.items = [ord("k")]
            self.get_calls = 0

        def qsize(self):
            return len(self.items)

        def get_nowait(self):
            self.get_calls += 1
            if self.get_calls > 3:
                raise RuntimeError("drain starved simulation loop")
            value = self.items.pop(0)
            self.items.append(ord("k"))
            return value

    events = GrowingQueue()
    state = mujoco_viewer.drain_key_events(events, mujoco_viewer.ViewerControlState())
    assert state.joint_jog_direction == 1
    assert events.get_calls == 1
    assert events.items == [ord("k")]


def test_interleaved_jog_selection_jog_selects_new_joint_without_target_jump():
    sim = FakeSim()
    events = SimpleQueue()
    for key in (ord("k"), ord("x"), ord("k")):
        events.put(key)
    state = mujoco_viewer.process_key_events(
        sim, events, mujoco_viewer.ViewerControlState(), 0.05, 0.005
    )
    assert sim.targets[:2] == pytest.approx([0.0, 0.0])
    assert state.selected_joint == 1
    assert state.joint_jog_direction == 1


def test_interleaved_open_reset_close_latches_close_after_reset():
    sim = FakeSim()
    events = SimpleQueue()
    for key in (ord("o"), ord("r"), ord("c")):
        events.put(key)
    state = mujoco_viewer.process_key_events(
        sim, events, mujoco_viewer.ViewerControlState(gripper_width=0.05), 0.05, 0.005
    )
    assert ("reset",) in sim.calls
    assert state.gripper_width == pytest.approx(0.09)
    assert state.gripper_jog_direction == -1


def test_pause_single_step_resume_executes_step_at_its_ordered_position():
    sim = FakeSim()
    events = SimpleQueue()
    for key in (ord(" "), ord("."), ord(" ")):
        events.put(key)
    state = mujoco_viewer.process_key_events(
        sim, events, mujoco_viewer.ViewerControlState(), 0.05, 0.005
    )
    assert sum(call == ("step",) for call in sim.calls) == 1
    assert state.paused is False
    assert state.single_step is False


def test_quit_discards_later_jog_from_same_snapshot():
    sim = FakeSim()
    events = SimpleQueue()
    for key in (ord("q"), ord("k")):
        events.put(key)
    state = mujoco_viewer.process_key_events(
        sim, events, mujoco_viewer.ViewerControlState(), 0.05, 0.005
    )
    assert state.quit is True
    assert not any(call[0] == "joints" for call in sim.calls)
    assert events.empty()


def test_escape_discards_later_reset_from_same_snapshot():
    sim = FakeSim()
    events = SimpleQueue()
    for key in (256, ord("r")):
        events.put(key)
    state = mujoco_viewer.process_key_events(
        sim, events, mujoco_viewer.ViewerControlState(), 0.05, 0.005
    )
    assert state.quit is True
    assert ("reset",) not in sim.calls
    assert events.empty()


def test_pause_toggles_and_single_step_only_queues_while_paused():
    running = mujoco_viewer.reduce_key(mujoco_viewer.ViewerControlState(), ".")
    assert running.single_step is False
    paused = mujoco_viewer.reduce_key(running, " ")
    assert paused.paused is True
    assert mujoco_viewer.reduce_key(paused, ".").single_step is True
    assert mujoco_viewer.reduce_key(paused, " ").paused is False


def test_apply_pending_commands_uses_core_clamping_and_clears_one_shots():
    sim = FakeSim()
    sim.targets[0] = 0.95
    state = mujoco_viewer.ViewerControlState(
        joint_targets=(0.95, 0.0, 0.0, 0.0, 0.0, 0.0),
        gripper_width=0.05,
        joint_delta=1,
        gripper_delta=1,
    )
    state = mujoco_viewer.apply_pending_commands(sim, state, 0.2, 0.1)
    assert sim.targets[0] == 1.0
    assert sim.width == 0.09
    assert state.joint_targets[0] == 1.0
    assert state.gripper_width == 0.09
    assert state.joint_delta == state.gripper_delta == 0


def test_reset_preserves_pause_and_resynchronizes_targets():
    sim = FakeSim()
    state = mujoco_viewer.apply_pending_commands(
        sim, mujoco_viewer.ViewerControlState(paused=True, reset=True), 0.1, 0.01
    )
    assert ("reset",) in sim.calls
    assert state.paused is True
    assert state.reset is False
    assert state.gripper_width == 0.09


def test_reset_then_jog_in_same_event_batch_applies_jog_after_reset():
    sim = FakeSim()
    sim.targets[0] = 0.8
    state = mujoco_viewer.ViewerControlState(reset=True, joint_delta=2)
    state = mujoco_viewer.apply_pending_commands(sim, state, 0.1, 0.01)
    assert sim.calls[0] == ("reset",)
    assert ("joints", {"joint1": 0.2}) in sim.calls
    assert state.joint_targets[0] == pytest.approx(0.2)


def test_overlay_contains_selected_target_gripper_pause_and_help():
    state = mujoco_viewer.ViewerControlState(
        selected_joint=2,
        joint_targets=(0.0, 0.0, 0.25, 0.0, 0.0, 0.0),
        joint_positions=(0.0, 0.0, 0.20, 0.0, 0.0, 0.0),
        joint_velocities=(0.0, 0.0, 0.03, 0.0, 0.0, 0.0),
        gripper_width=0.04,
        gripper_actual_width=0.035,
        max_contact_force=2.5,
        contact_count=2,
        paused=True,
        active_mode="gravity_comp",
        requested_torques=(0.0, 0.0, 1.5, 0.0, 0.0, 0.0),
        applied_torques=(0.0, 0.0, 1.2, 0.0, 0.0, 0.0),
        saturated=(False, False, True, False, False, False),
        collision_visible=True,
    )
    text = mujoco_viewer.overlay_text(state)
    for expected in (
        "GRAVITY_COMP", "J3", "+0.200", "+0.030", "+0.250", "0.035", "0.040",
        "CONTACTS", "2.50 N", "PAUSED", "+1.50/ +1.20", "!", "Z/X",
        "J/K start jog", "SHOWN",
    ):
        assert expected in text


def test_overlay_has_clear_sections_and_all_six_joints():
    state = mujoco_viewer.ViewerControlState(selected_joint=4)
    system = mujoco_viewer.system_panel_text(state)
    joints = mujoco_viewer.joint_panel_text(state)
    gripper = mujoco_viewer.gripper_panel_text(state)

    assert "MODE" in system[0]
    assert "captures and holds pose" in system[1]
    for name in ("J1", "J2", "J3", "J4", "J5", "J6"):
        assert name in joints[0]
    assert "> J5" in joints[0]
    assert "ACTUAL WIDTH" in gripper[0]
    controls = mujoco_viewer.controls_panel_text()
    assert "S" in controls[0]
    assert "STOP + hold" in controls[1]


def test_overlay_panels_remain_compact_for_small_vm_windows():
    state = mujoco_viewer.ViewerControlState()

    for left, right in (
        mujoco_viewer.system_panel_text(state),
        mujoco_viewer.joint_panel_text(state),
        mujoco_viewer.gripper_panel_text(state),
        mujoco_viewer.controls_panel_text(),
    ):
        left_width = max(map(len, left.splitlines()))
        right_width = max(map(len, right.splitlines()))
        assert left_width + right_width <= 55


def test_command_events_set_joint_targets_gripper_and_mode_on_sim_thread():
    sim = FakeSim()
    events = SimpleQueue()
    for line in (
        "joints 0.1 -0.2 -0.3 0.4 0.5 0.6",
        "gripper 0.04",
        "mode hold",
    ):
        events.put(line)
    status = io.StringIO()

    state = mujoco_viewer.process_command_events(
        sim,
        events,
        mujoco_viewer.ViewerControlState(
            jog_speed_index=2,
            joint_jog_rate=0.3,
            gripper_jog_rate=0.04,
        ),
        status,
    )

    assert sim.targets == pytest.approx([0.1, -0.2, -0.3, 0.4, 0.5, 0.6])
    assert sim.width == pytest.approx(0.04)
    assert sim.control_mode == "hold"
    assert state.joint_targets == pytest.approx(tuple(sim.targets))
    assert state.gripper_width == pytest.approx(0.04)
    assert state.active_mode == "hold"
    assert state.jog_speed_index == 2
    assert state.joint_jog_rate == pytest.approx(0.3)
    assert state.gripper_jog_rate == pytest.approx(0.04)
    assert status.getvalue().count("command result:") == 3


def test_command_events_report_errors_and_keep_running():
    sim = FakeSim()
    events = SimpleQueue()
    events.put("joints 1 2")
    events.put("joint joint1 0.2")
    status = io.StringIO()

    state = mujoco_viewer.process_command_events(
        sim,
        events,
        mujoco_viewer.ViewerControlState(),
        status,
    )

    assert "command error: usage: joints" in status.getvalue()
    assert sim.targets[0] == pytest.approx(0.2)
    assert state.quit is False


def test_continuous_jog_advances_target_until_explicit_stop():
    sim = FakeSim()
    state = mujoco_viewer.ViewerControlState(
        joint_jog_direction=1,
        jog_time_remaining=0.02,
    )
    state = mujoco_viewer.apply_continuous_jog(
        sim,
        state,
        dt=0.01,
        joint_rate=0.4,
        gripper_rate=0.03,
    )
    assert sim.targets[0] == pytest.approx(0.004)
    assert state.joint_jog_direction == 1

    state = mujoco_viewer.apply_continuous_jog(
        sim,
        state,
        dt=0.01,
        joint_rate=0.4,
        gripper_rate=0.03,
    )
    assert sim.targets[0] == pytest.approx(0.008)
    assert state.joint_jog_direction == 1
    assert state.jog_time_remaining == pytest.approx(0.0)

    state = mujoco_viewer.reduce_key(state, "s")
    state = mujoco_viewer.apply_pending_commands(sim, state, 0.01, 0.001)
    assert state.joint_jog_direction == 0
    assert sim.control_mode == "hold"


def test_fast_gear_scales_joint_and_gripper_jog_rates():
    sim = FakeSim()
    state = mujoco_viewer.ViewerControlState(
        joint_jog_direction=1,
        jog_speed_index=2,
        gripper_width=0.05,
    )
    state = mujoco_viewer.apply_continuous_jog(
        sim,
        state,
        dt=0.01,
        joint_rate=0.4,
        gripper_rate=0.03,
    )
    assert sim.targets[0] == pytest.approx(0.01)

    state = replace(state, joint_jog_direction=0, gripper_jog_direction=-1)
    state = mujoco_viewer.apply_continuous_jog(
        sim,
        state,
        dt=0.01,
        joint_rate=0.4,
        gripper_rate=0.03,
    )
    assert sim.width == pytest.approx(0.04925)


def test_tab_cycles_joint_xyz_rpy_and_axis_selection_is_independent():
    state = mujoco_viewer.ViewerControlState(selected_joint=2)
    state = mujoco_viewer.reduce_key(state, "tab")
    assert state.interaction_mode == "xyz"
    state = mujoco_viewer.reduce_key(state, "x")
    assert state.selected_cartesian_axis == 1
    assert state.selected_joint == 2
    state = mujoco_viewer.reduce_key(state, "tab")
    assert state.interaction_mode == "rpy"
    state = mujoco_viewer.reduce_key(state, "tab")
    assert state.interaction_mode == "joint"


def test_special_keys_toggle_plots_record_replay_and_clear():
    state = mujoco_viewer.ViewerControlState()
    for keycode, field in ((291, "plots_visible"), (292, "record_toggle"),
                           (293, "replay_toggle"), (294, "trajectory_clear")):
        state = mujoco_viewer.reduce_key(state, mujoco_viewer._decode_key(keycode))
        assert getattr(state, field) is True


def test_cartesian_jog_is_rate_limited_and_uses_selected_axis():
    sim = FakeSim()

    class FakeCartesian:
        calls = []

        def command_delta(self, delta):
            self.calls.append(delta)
            return SimpleNamespace(
                success=True,
                status="converged",
                position_error_m=0.0001,
                orientation_error_rad=0.0,
                joint_positions=(0.1,) * 6,
            )

    controller = FakeCartesian()
    state = mujoco_viewer.ViewerControlState(
        interaction_mode="xyz", selected_cartesian_axis=1, joint_jog_direction=1
    )
    state = mujoco_viewer.apply_continuous_jog(
        sim, state, dt=0.01, joint_rate=0.2, gripper_rate=0.02,
        cartesian_controller=controller,
    )
    assert controller.calls == []
    state = mujoco_viewer.apply_continuous_jog(
        sim, state, dt=0.01, joint_rate=0.2, gripper_rate=0.02,
        cartesian_controller=controller,
    )
    assert controller.calls[0].xyz_m == pytest.approx((0.0, 0.0004, 0.0))
    assert state.ik_status == "converged"


def test_draggable_target_aligns_with_end_effector_and_submits_changed_pose():
    mujoco = pytest.importorskip("mujoco")
    from pathlib import Path

    scene = Path(__file__).resolve().parents[1] / "src/rebotarm_simulation/models/rebotarm/scene.xml"
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    mujoco.mj_forward(model, data)
    mocap_id, position, quaternion = mujoco_viewer.align_cartesian_target(model, data)
    ee_site = model.site("ee_site").id
    assert position == pytest.approx(tuple(data.site_xpos[ee_site]))
    assert sum(value * value for value in quaternion) == pytest.approx(1.0)

    class Controller:
        calls = []

        def command_pose(self, pos, quat):
            self.calls.append((pos, quat))
            return SimpleNamespace(
                success=True, status="converged", joint_positions=(0.2,) * 6,
                position_error_m=0.0, orientation_error_rad=0.0,
            )

    controller = Controller()
    old_pose = (position, quaternion)
    data.mocap_pos[mocap_id, 0] += 0.005
    new_pose, state = mujoco_viewer.process_cartesian_target(
        controller, data, mocap_id, old_pose, mujoco_viewer.ViewerControlState()
    )
    assert new_pose != old_pose
    assert len(controller.calls) == 1
    assert state.joint_targets == pytest.approx((0.2,) * 6)


def test_parser_accepts_model_and_positive_steps():
    args = mujoco_viewer.build_parser().parse_args(
        [
            "--model", "scene.xml", "--joint-step", "0.2",
            "--gripper-step", "0.01", "--duration", "1.5",
        ]
    )
    assert (args.model, args.joint_step, args.gripper_step, args.duration) == (
        "scene.xml", 0.2, 0.01, 1.5,
    )
    assert args.no_command_input is False
    assert args.verbose_status is False
    assert args.joint_rate == pytest.approx(0.20)
    assert args.gripper_rate == pytest.approx(0.02)
    for flag in ("--joint-step", "--gripper-step", "--duration"):
        with pytest.raises(SystemExit):
            mujoco_viewer.build_parser().parse_args([flag, "0"])


def test_parser_enables_terminal_state_stream_only_on_request():
    args = mujoco_viewer.build_parser().parse_args(["--verbose-status"])
    assert args.verbose_status is True


def test_viewer_launch_hides_only_known_wayland_window_position_warning():
    def launch(*_args, **_kwargs):
        warnings.warn(
            "(65548) b'Wayland: The platform does not provide the window position'",
            UserWarning,
        )
        warnings.warn("different GLFW problem", UserWarning)
        return object()

    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        viewer = mujoco_viewer._launch_passive_viewer(
            launch, object(), object(), lambda _keycode: None
        )

    assert viewer is not None
    assert [str(item.message) for item in caught] == ["different GLFW problem"]


@pytest.mark.parametrize("value", ["-1", "nan", "inf"])
def test_parser_rejects_invalid_duration(value):
    with pytest.raises(SystemExit):
        mujoco_viewer.build_parser().parse_args(["--duration", value])


def test_runtime_launches_passively_steps_syncs_sleeps_and_always_closes():
    sim = FakeSim("scene.xml")
    holder = {}

    def launch(model, data, *, key_callback, show_left_ui, show_right_ui):
        assert (model, data) == (sim.viewer_model, sim.viewer_data)
        assert show_left_ui is False
        assert show_right_ui is False
        holder["viewer"] = FakeViewer(key_callback, [ord("k"), ord(" "), ord("."), ord("q")])
        return holder["viewer"]

    sleeps = []
    status = io.StringIO()
    clock_values = iter((0.0, 0.002, 1.0, 1.002, 2.0, 2.002))
    code = mujoco_viewer.main(
        ["--model", "scene.xml"],
        sim_factory=lambda _: sim,
        launch_passive=launch,
        sleep=sleeps.append,
        clock=lambda: next(clock_values),
        status_stream=status,
    )
    assert code == 0
    assert ("joints", {"joint1": pytest.approx(0.002)}) in sim.calls
    assert sum(call == ("step",) for call in sim.calls) == 2
    assert holder["viewer"].sync_count >= 3
    output = status.getvalue()
    assert "viewer ready" in output
    assert "joint1" not in output
    assert holder["viewer"].opt.geomgroup[2] == 1
    assert holder["viewer"].opt.geomgroup[3] == 0
    assert len(holder["viewer"].texts) == 4
    assert sleeps == pytest.approx([0.008, 0.008, 0.008])
    assert holder["viewer"].closed is True
    assert sim.closed is True


def test_runtime_accepts_terminal_joint_commands_while_viewer_runs():
    sim = FakeSim("scene.xml")
    holder = {}

    class ThreeCycleViewer:
        def __init__(self, key_callback):
            self.key_callback = key_callback
            self.cycles = 0
            self.closed = False

        def is_running(self):
            self.cycles += 1
            return self.cycles <= 3

        def sync(self):
            pass

        def close(self):
            self.closed = True

    def launch(model, data, *, key_callback, show_left_ui, show_right_ui):
        assert show_left_ui is False
        assert show_right_ui is False
        holder["viewer"] = ThreeCycleViewer(key_callback)
        return holder["viewer"]

    status = io.StringIO()
    code = mujoco_viewer.main(
        [],
        sim_factory=lambda _: sim,
        launch_passive=launch,
        sleep=lambda _: None,
        status_stream=status,
        command_stream=io.StringIO("joints 0.1 -0.2 -0.3 0.4 0.5 0.6\ngripper 0.04\n"),
    )

    assert code == 0
    assert sim.targets == pytest.approx([0.1, -0.2, -0.3, 0.4, 0.5, 0.6])
    assert sim.width == pytest.approx(0.04)
    assert "command result:" in status.getvalue()
    assert holder["viewer"].closed is True
    assert sim.closed is True


def test_duration_stops_at_requested_simulation_time_and_closes():
    sim = FakeSim()

    class RunningViewer:
        closed = False

        def is_running(self):
            return True

        def sync(self):
            pass

        def close(self):
            self.closed = True

    viewer = RunningViewer()
    assert mujoco_viewer.main(
        ["--duration", "0.025"],
        sim_factory=lambda _: sim,
        launch_passive=lambda *_args, **_kwargs: viewer,
        sleep=lambda _: None,
        status_stream=io.StringIO(),
    ) == 0
    assert sim.time == pytest.approx(0.03)
    assert viewer.closed is True
    assert sim.closed is True


def test_keyboard_interrupt_from_sync_returns_130_and_closes_without_escaping():
    sim = FakeSim()

    class InterruptingViewer:
        closed = False

        def is_running(self):
            return True

        def sync(self):
            raise KeyboardInterrupt

        def close(self):
            self.closed = True

    viewer = InterruptingViewer()
    assert mujoco_viewer.main(
        [],
        sim_factory=lambda _: sim,
        launch_passive=lambda *_args, **_kwargs: viewer,
        sleep=lambda _: None,
        status_stream=io.StringIO(),
    ) == 130
    assert viewer.closed is True
    assert sim.closed is True


def test_callback_from_other_thread_only_enqueues_until_main_loop_drains():
    sim = FakeSim()
    main_thread = threading.get_ident()

    class ConcurrentViewer:
        def __init__(self, callback):
            self.callback = callback
            self.iteration = 0

        def is_running(self):
            self.iteration += 1
            return self.iteration <= 2

        def sync(self):
            worker = threading.Thread(target=lambda: (self.callback(ord("k")), self.callback(ord("k"))))
            worker.start()
            worker.join()

        def close(self):
            pass

    assert mujoco_viewer.main(
        [],
        sim_factory=lambda _: sim,
        launch_passive=lambda *_args, key_callback, show_left_ui, show_right_ui: ConcurrentViewer(key_callback),
        sleep=lambda _: None,
        status_stream=io.StringIO(),
    ) == 0
    assert sim.targets[0] == pytest.approx(0.002)
    assert sim.call_threads and set(sim.call_threads) == {main_thread}


def test_runtime_closes_sim_when_viewer_launch_fails():
    sim = FakeSim()

    def broken(*_args, **_kwargs):
        raise RuntimeError("display unavailable")

    with pytest.raises(RuntimeError, match="display unavailable"):
        mujoco_viewer.main([], sim_factory=lambda _: sim, launch_passive=broken)
    assert sim.closed is True


def test_runtime_terminal_is_quiet_when_viewer_is_already_closed():
    sim = FakeSim()
    status = io.StringIO()

    class ClosedViewer:
        def is_running(self):
            return False

        def close(self):
            pass

    assert mujoco_viewer.main(
        [],
        sim_factory=lambda _: sim,
        launch_passive=lambda *_args, **_kwargs: ClosedViewer(),
        status_stream=status,
    ) == 0
    assert status.getvalue().count("\n") == 1
    assert "viewer ready" in status.getvalue()
    assert "joint1" not in status.getvalue()


def test_runtime_closes_sim_even_when_viewer_close_raises():
    sim = FakeSim()

    class BadCloseViewer:
        m = None

        def is_running(self):
            return False

        def close(self):
            raise RuntimeError("viewer close failed")

    with pytest.raises(RuntimeError, match="viewer close failed"):
        mujoco_viewer.main(
            [],
            sim_factory=lambda _: sim,
            launch_passive=lambda *_args, **_kwargs: BadCloseViewer(),
            status_stream=io.StringIO(),
        )
    assert sim.closed is True


def test_cleanup_waits_for_public_viewer_model_to_clear_before_sim_close():
    order = []

    class OrderedSim(FakeSim):
        def close(self):
            order.append("sim.close")
            assert viewer.m is None
            super().close()

    class TransitioningViewer:
        def __init__(self):
            self.closed = False
            self.polls = 0

        @property
        def m(self):
            self.polls += 1
            return object() if self.polls < 2 else None

        def close(self):
            order.append("viewer.close")
            self.closed = True

    sim = OrderedSim()
    viewer = TransitioningViewer()
    sleeps = []
    mujoco_viewer._close_viewer_then_sim(
        viewer,
        sim,
        object(),
        object(),
        clock=iter((0.0, 0.1, 0.2)).__next__,
        sleep=sleeps.append,
        timeout=1.0,
    )
    assert order == ["viewer.close", "sim.close"]
    assert sleeps == [0.01]
    assert sim.closed is True


def test_cleanup_timeout_retains_unsafe_handles_and_does_not_close_sim():
    sim = FakeSim()

    class StuckViewer:
        m = object()

        def close(self):
            pass

    viewer = StuckViewer()
    retained_before = len(mujoco_viewer._RETAINED_UNSAFE_VIEWERS)
    with pytest.raises(TimeoutError, match="did not finish"):
        mujoco_viewer._close_viewer_then_sim(
            viewer,
            sim,
            object(),
            object(),
            clock=iter((0.0, 2.0)).__next__,
            sleep=lambda _: None,
            timeout=1.0,
        )
    assert sim.closed is False
    assert len(mujoco_viewer._RETAINED_UNSAFE_VIEWERS) == retained_before + 1


def test_close_error_with_live_public_model_retains_handles_without_sim_close():
    sim = FakeSim()

    class LiveViewer:
        m = object()

        def close(self):
            raise RuntimeError("close failed")

    viewer = LiveViewer()
    retained_before = len(mujoco_viewer._RETAINED_UNSAFE_VIEWERS)
    with pytest.raises(RuntimeError, match="close failed"):
        mujoco_viewer._close_viewer_then_sim(viewer, sim, object(), object())
    assert sim.closed is False
    assert len(mujoco_viewer._RETAINED_UNSAFE_VIEWERS) == retained_before + 1


def test_close_error_with_cleared_public_model_safely_closes_sim_then_reraises():
    sim = FakeSim()

    class ClearedViewer:
        m = None

        def close(self):
            raise RuntimeError("close failed")

    with pytest.raises(RuntimeError, match="close failed"):
        mujoco_viewer._close_viewer_then_sim(
            ClearedViewer(), sim, object(), object()
        )
    assert sim.closed is True


def test_close_error_with_unreadable_public_model_conservatively_retains_handles():
    sim = FakeSim()

    class UnreadableViewer:
        @property
        def m(self):
            raise ValueError("model unavailable")

        def close(self):
            raise RuntimeError("close failed")

    viewer = UnreadableViewer()
    retained_before = len(mujoco_viewer._RETAINED_UNSAFE_VIEWERS)
    with pytest.raises(RuntimeError, match="close failed"):
        mujoco_viewer._close_viewer_then_sim(viewer, sim, object(), object())
    assert sim.closed is False
    assert len(mujoco_viewer._RETAINED_UNSAFE_VIEWERS) == retained_before + 1
