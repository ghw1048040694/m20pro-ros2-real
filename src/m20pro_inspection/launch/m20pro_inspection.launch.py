import os
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration("config_file").perform(context)
    pythonpath = LaunchConfiguration("pythonpath").perform(context).strip()
    ld_preload = LaunchConfiguration("ld_preload").perform(context).strip()
    existing_pythonpath = os.environ.get("PYTHONPATH", "")
    existing_ld_preload = os.environ.get("LD_PRELOAD", "")
    node_env = {}
    if pythonpath:
        node_env["PYTHONPATH"] = (
            pythonpath if not existing_pythonpath else pythonpath + os.pathsep + existing_pythonpath
        )
    if ld_preload and os.path.exists(ld_preload):
        node_env["LD_PRELOAD"] = (
            ld_preload if not existing_ld_preload else ld_preload + os.pathsep + existing_ld_preload
        )
    overrides = {
        "m20pro_yolov8_inspection": {
            "ros__parameters": {
                "model_path": LaunchConfiguration("model_path").perform(context),
                "class_names_path": LaunchConfiguration("class_names_path").perform(context),
                "backend": LaunchConfiguration("backend").perform(context),
                "source_type": LaunchConfiguration("source_type").perform(context),
                "rtsp_url": LaunchConfiguration("rtsp_url").perform(context),
                "image_topic": LaunchConfiguration("image_topic").perform(context),
                "camera_name": LaunchConfiguration("camera_name").perform(context),
            }
        }
    }
    override_file = tempfile.NamedTemporaryFile(
        mode="w",
        prefix="m20pro_inspection_params_",
        suffix=".yaml",
        delete=False,
    )
    with override_file:
        yaml.safe_dump(overrides, override_file, allow_unicode=True, sort_keys=False)

    return [
        Node(
            package="m20pro_inspection",
            executable="yolov8_inspection",
            name="m20pro_yolov8_inspection",
            output="screen",
            parameters=[config_file, override_file.name],
            additional_env=node_env,
        )
    ]


def generate_launch_description():
    inspection_share = get_package_share_directory("m20pro_inspection")
    default_config = os.path.join(inspection_share, "config", "yolov8_inspection.yaml")
    default_model = os.path.join(inspection_share, "models", "best_rk3588_fp16.rknn")
    default_classes = os.path.join(inspection_share, "models", "labels_zh.txt")

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("model_path", default_value=default_model),
        DeclareLaunchArgument("class_names_path", default_value=default_classes),
        DeclareLaunchArgument("backend", default_value="auto"),
        DeclareLaunchArgument("source_type", default_value="rtsp"),
        DeclareLaunchArgument("rtsp_url", default_value="rtsp://10.21.31.103:8554/video1"),
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("camera_name", default_value="front_wide"),
        DeclareLaunchArgument("pythonpath", default_value="/home/user/m20pro_rknn_pydeps"),
        DeclareLaunchArgument("ld_preload", default_value=""),
        OpaqueFunction(function=_launch_setup),
    ])
