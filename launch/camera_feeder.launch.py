# Launch the syncai_camera_feeder node — the sole opener of /dev/syncai/camera0, fanning its MJPEG
# stream out to the WebRTC worker (RTP/JPEG on 127.0.0.1) and to ROS (image_raw/compressed). See
# PROPOSAL.md for the design.
#
# robot_id is read from the system config INI at launch time (same convention as
# robot_state.launch.py) and is used as the node namespace and to prefix frame_id, since TF frame
# names are not namespaced by ROS.

import configparser
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch import logging as launch_logging
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Absolute path so the INI resolves no matter what cwd the launch is started from. ~/robot_ws is
# the workspace inside the robot container, where docker-compose bind-mounts the per-robot instance
# INI over config/system.ini.
DEFAULT_SYSTEM_INI = os.path.expanduser("~/robot_ws/config/system.ini")
FALLBACK_ROBOT_ID = "default_robot"

logger = launch_logging.get_logger("camera_feeder.launch")


def read_robot_id(config_path: str) -> str:
    config = configparser.ConfigParser()
    if not config.read(config_path):
        logger.warning(
            f"System config '{config_path}' not found; "
            f"falling back to robot_id '{FALLBACK_ROBOT_ID}'"
        )
        return FALLBACK_ROBOT_ID

    robot_id = config.get("system", "robot_id", fallback="").strip()
    if not robot_id:
        logger.warning(
            f"No [system] robot_id in '{config_path}'; "
            f"falling back to '{FALLBACK_ROBOT_ID}'"
        )
        return FALLBACK_ROBOT_ID

    return robot_id


def launch_setup(context, *args, **kwargs):
    # LaunchConfiguration values only resolve inside an OpaqueFunction, and we need the resolved
    # robot_id here to namespace the node and its frame.
    config_path = LaunchConfiguration("system_config").perform(context)
    robot_id = read_robot_id(config_path)

    params_file = LaunchConfiguration("params_file")

    # Later entries in this list take precedence over the params file.
    overrides = {
        "frame_id": f"{robot_id}/camera_optical_frame",
    }

    # No `name=`: the node is constructed as "syncai_camera_feeder", and the params file uses a
    # bare /** key (rclrs does not expand /**/syncai_camera_feeder-style wildcards).
    #
    # respawn: this node is the only thing holding the camera, so if it dies both the WebRTC view
    # and the recorded topic go dark until it is back (PROPOSAL.md §8 item 3).
    camera_feeder_node = Node(
        package="syncai_camera_feeder",
        executable="camera_feeder_node",
        namespace=robot_id,
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            params_file,
            overrides,
        ],
    )

    return [camera_feeder_node]


def generate_launch_description():
    pkg_share = get_package_share_directory("syncai_camera_feeder")
    default_params_file = os.path.join(pkg_share, "params", "camera_feeder_params.yaml")

    declare_system_config = DeclareLaunchArgument(
        "system_config",
        default_value=DEFAULT_SYSTEM_INI,
        description="Path to the system INI file providing [system] robot_id",
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Full path to the camera_feeder parameters YAML file",
    )

    return LaunchDescription(
        [
            declare_system_config,
            declare_params_file,
            OpaqueFunction(function=launch_setup),
        ]
    )
