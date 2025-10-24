from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    share = Path(get_package_share_directory("f_human_detection2"))
    rviz_cfg = str(share / "rviz" / "pose_debug.rviz")
    dev_yaml = str(share / "config" / "dev.yaml")

    return LaunchDescription([
        Node(
            package="f_human_detection2",
            executable="pose_node",
            name="f_human_detection2",
            parameters=[dev_yaml]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_cfg]
        )
    ])
