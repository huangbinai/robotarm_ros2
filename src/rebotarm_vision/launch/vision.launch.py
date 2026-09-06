from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

from rebotarm_calibration.handeye_config import load_handeye_config


def _launch_setup(context):
    camera_config = str(
        Path(LaunchConfiguration("camera_config").perform(context)).expanduser()
    )
    handeye_config = Path(
        LaunchConfiguration("handeye_config").perform(context)
    ).expanduser()
    yolo_model_path = LaunchConfiguration("yolo_model_path").perform(context).strip()
    yolo_device = LaunchConfiguration("yolo_device").perform(context).strip()
    handeye = load_handeye_config(handeye_config)

    vision_overrides = {}
    if yolo_model_path:
        vision_overrides["yolo.model_path"] = yolo_model_path
    if yolo_device:
        vision_overrides["yolo.device"] = yolo_device

    common_environment = {
        "QT_QPA_PLATFORM": "xcb",
        "QT_QPA_FONTDIR": "/usr/share/fonts/truetype/dejavu",
    }
    python_prefix = LaunchConfiguration("vision_python_executable")
    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="rebotarm_handeye_static_tf",
            output="screen",
            arguments=handeye.as_static_transform_arguments(),
        ),
        Node(
            package="rebotarm_vision",
            executable="rebotarm_vision_node",
            prefix=python_prefix,
            name="rebotarm_vision_node",
            output="screen",
            parameters=[camera_config, vision_overrides],
            additional_env=common_environment,
        ),
        Node(
            package="rebotarm_vision",
            executable="rebotarm_grasp_tcp_frame",
            prefix=python_prefix,
            name="rebotarm_grasp_tcp_frame",
            output="screen",
            parameters=[camera_config],
            additional_env=common_environment,
        ),
    ]


def generate_launch_description():
    vision_share = Path(get_package_share_directory("rebotarm_vision"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_config",
                default_value=str(vision_share / "config" / "camera.yaml"),
            ),
            DeclareLaunchArgument(
                "handeye_config",
                default_value=str(vision_share / "config" / "handeye.yaml"),
            ),
            DeclareLaunchArgument(
                "vision_python_executable",
                default_value=EnvironmentVariable(
                    "REBOTARM_VISION_PYTHON",
                    default_value="python3",
                ),
            ),
            DeclareLaunchArgument("yolo_model_path", default_value=""),
            DeclareLaunchArgument("yolo_device", default_value=""),
            OpaqueFunction(function=_launch_setup),
        ]
    )
