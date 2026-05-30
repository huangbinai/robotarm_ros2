import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _bool_text(value: str) -> str:
    return "true" if str(value).strip().lower() in {"1", "true", "yes", "on"} else "false"


def _as_bool(value: str) -> bool:
    return _bool_text(value) == "true"


def _record_path(name: str, explicit_path: str) -> str:
    if explicit_path:
        return explicit_path
    safe_name = name.strip() or "teach_record"
    safe_name = os.path.basename(safe_name.replace("\\", "/")) or "teach_record"
    if not safe_name.endswith(".jsonl"):
        safe_name = f"{safe_name}.jsonl"
    return f"teleop_records/{safe_name}"


def _load_profile(profile_name: str) -> dict:
    config_path = os.path.join(
        get_package_share_directory("rebotarm_bringup"),
        "config",
        "replay_profiles.yaml",
    )
    with open(config_path, "r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    profiles = config.get("profiles", {})
    if profile_name not in profiles:
        available = ", ".join(sorted(profiles))
        raise RuntimeError(f"profile must be one of: {available}")
    return dict(profiles[profile_name])


def _panel_node(
    *,
    teleop_config,
    arm_namespace: str,
    record_path: str,
    use_hardware: str,
    web_execute_enabled: str,
    panel_mode: str,
):
    return Node(
        package="rebotarm_interactive_control",
        executable="TeleopStatusPanelNode",
        name="teleop_status_panel_node",
        output="screen",
        parameters=[
            teleop_config,
            {
                "arm_namespace": arm_namespace,
                "web_execute_enabled": _as_bool(web_execute_enabled),
                "record_path": record_path,
                "use_hardware": _as_bool(use_hardware),
                "panel_mode": panel_mode,
            },
        ],
    )


def _replay_node(*, teleop_config, arm_namespace: str, record_path: str, profile: dict, dry_run: str):
    params = dict(profile)
    params.update(
        {
            "arm_namespace": arm_namespace,
            "record_path": record_path,
            "dry_run": _as_bool(dry_run),
            "moveit_planning_service": "/plan_kinematic_path",
            "collision_check_service": "/check_state_validity",
        }
    )
    return Node(
        package="rebotarm_interactive_control",
        executable="TeachReplayNode",
        name="teach_replay_node",
        output="screen",
        parameters=[teleop_config, params],
    )


def _keyboard_node(*, teleop_config, arm_namespace: str, keyboard_prefix: str):
    return Node(
        package="rebotarm_interactive_control",
        executable="TeleopKeyboardNode",
        name="teleop_keyboard_node",
        output="screen",
        prefix=keyboard_prefix,
        parameters=[teleop_config, {"arm_namespace": arm_namespace}],
    )


def _launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context).strip().lower()
    profile_name = LaunchConfiguration("profile").perform(context).strip().lower()
    name = LaunchConfiguration("name").perform(context).strip()
    explicit_record_path = LaunchConfiguration("record_path").perform(context).strip()
    record_path = _record_path(name, explicit_record_path)
    arm_namespace = LaunchConfiguration("arm_namespace").perform(context)
    channel = LaunchConfiguration("channel").perform(context)
    use_hardware = _bool_text(LaunchConfiguration("use_hardware").perform(context))
    use_rviz = _bool_text(LaunchConfiguration("use_rviz").perform(context))
    panel = _bool_text(LaunchConfiguration("panel").perform(context))
    web_execute_enabled = _bool_text(LaunchConfiguration("web_execute_enabled").perform(context))
    keyboard_prefix = LaunchConfiguration("keyboard_prefix").perform(context)
    teleop_config = LaunchConfiguration("teleop_config")
    bringup_share = get_package_share_directory("rebotarm_bringup")
    moveit_launch = os.path.join(bringup_share, "launch", "moveit_hardware.launch.py")
    teleop_launch = os.path.join(bringup_share, "launch", "teleop_system.launch.py")

    if mode not in {"app", "teleop", "record", "check", "replay"}:
        raise RuntimeError("mode must be one of: app, teleop, record, check, replay")

    if mode == "teleop":
        return [
            LogInfo(msg="reBotArm app mode=teleop: starting lightweight keyboard/web/RViz system"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(teleop_launch),
                launch_arguments={
                    "arm_namespace": arm_namespace,
                    "use_hardware": use_hardware,
                    "use_local_rviz": use_rviz,
                    "panel": panel,
                    "web_execute_enabled": web_execute_enabled,
                    "record": "false",
                    "record_path": record_path,
                    "teleop_config": teleop_config,
                    "keyboard_prefix": keyboard_prefix,
                }.items(),
            ),
        ]

    if mode == "app":
        if use_hardware != "true":
            raise RuntimeError(
                "app mode is the full hardware + MoveIt workbench and requires use_hardware:=true; "
                "use mode:=teleop or teleop_system.launch.py for no-hardware tests"
            )
        actions = [
            LogInfo(msg="reBotArm app mode=app: starting full MoveIt + web teleop workbench"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(moveit_launch),
                launch_arguments={
                    "arm_namespace": arm_namespace,
                    "channel": channel,
                    "use_rviz": use_rviz,
                    "teach_record_path": record_path,
                }.items(),
            ),
            _keyboard_node(
                teleop_config=teleop_config,
                arm_namespace=arm_namespace,
                keyboard_prefix=keyboard_prefix,
            ),
            LogInfo(msg="teach recording services are provided by the controller internal recorder"),
        ]
        if panel == "true":
            actions.append(
                _panel_node(
                    teleop_config=teleop_config,
                    arm_namespace=arm_namespace,
                    record_path=record_path,
                    use_hardware=use_hardware,
                    web_execute_enabled=web_execute_enabled,
                    panel_mode="control",
                )
            )
        return actions

    if use_hardware != "true":
        raise RuntimeError(
            "record/check/replay app modes require use_hardware:=true; "
            "use teleop_system.launch.py or teach_replay.launch.py for no-hardware tests"
        )

    profile = _load_profile(profile_name)
    mode_web_execute_enabled = "false" if mode == "check" else web_execute_enabled
    panel_mode = "check" if mode == "check" else "control"
    actions = [
        LogInfo(
            msg=(
                f"reBotArm app mode={mode}, profile={profile_name}, "
                f"record_path={record_path}; Ctrl+C stops this launch"
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_launch),
            launch_arguments={
                "arm_namespace": arm_namespace,
                "channel": channel,
                "use_rviz": use_rviz,
                "teach_record_path": record_path,
            }.items(),
        ),
    ]

    if panel == "true":
        actions.append(
            _panel_node(
                teleop_config=teleop_config,
                arm_namespace=arm_namespace,
                record_path=record_path,
                use_hardware=use_hardware,
                web_execute_enabled=mode_web_execute_enabled,
                panel_mode=panel_mode,
            )
        )

    if mode == "record":
        actions.append(
            LogInfo(
                msg=(
                    "record mode uses the controller internal recorder; "
                    "start/stop recording from the web Teach Trajectory card"
                )
            )
        )
    elif mode == "check":
        actions.append(
            _replay_node(
                teleop_config=teleop_config,
                arm_namespace=arm_namespace,
                record_path=record_path,
                profile=profile,
                dry_run="true",
            )
        )
    elif mode == "replay":
        actions.append(
            _replay_node(
                teleop_config=teleop_config,
                arm_namespace=arm_namespace,
                record_path=record_path,
                profile=profile,
                dry_run=str(profile.get("dry_run", False)).lower(),
            )
        )

    return actions


def generate_launch_description():
    interactive_share = FindPackageShare("rebotarm_interactive_control")
    teleop_config = PathJoinSubstitution(
        [interactive_share, "config", "teleop_control.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="app"),
            DeclareLaunchArgument("profile", default_value="safe"),
            DeclareLaunchArgument("name", default_value="teach_record"),
            DeclareLaunchArgument("record_path", default_value=""),
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("channel", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("use_hardware", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("panel", default_value="true"),
            DeclareLaunchArgument("web_execute_enabled", default_value="true"),
            DeclareLaunchArgument(
                "keyboard_prefix",
                default_value="bash -lc 'exec \"$0\" \"$@\" < /dev/tty'",
            ),
            DeclareLaunchArgument("teleop_config", default_value=teleop_config),
            OpaqueFunction(function=_launch_setup),
        ]
    )
