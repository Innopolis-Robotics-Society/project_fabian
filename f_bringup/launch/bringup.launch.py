#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    camera_name_arg = DeclareLaunchArgument(
        "camera_name",
        default_value="maf_camera",
        description="Camera name for v4l2_camera",
    )

    image_source_arg = DeclareLaunchArgument(
        "image_source",
        default_value="/dev/video0",
        description="Video device path",
    )

    # Пути к конфигам human_detection2
    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("f_human_detection2"), "rviz", "pose_debug.rviz"]
    )
    params_yaml = PathJoinSubstitution(
        [FindPackageShare("f_human_detection2"), "config", "dev.yaml"]
    )

    # 1) Камера
    camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="v4l2_camera",
        parameters=[
            {"camera_name": LaunchConfiguration("camera_name")},
            {"video_device": LaunchConfiguration("image_source")},
            {"io_method": "read"},
            {"output_encoding": "rgb8"},
            {"frame_id": "camera"},
        ],
        output="screen",
    )

    # 2) TF (из f_bringup/tf.launch.py)
    tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("f_bringup"), "launch", "tf.launch.py"]
            )
        )
    )

    # 3) Pose estimation
    pose_node = Node(
        package="f_human_detection2",
        executable="pose_node",
        name="f_human_detection2",
        parameters=[params_yaml],
        output="screen",
    )

    # 4) Классификатор жестов
    gesture_classifier = Node(
        package="f_gesture_recognition",
        executable="pose_classifier",
        name="pose_classifier",
        output="screen",
    )

    # Узел команд (жест → текстовая команда)
    fox_command_node = Node(
        package="f_fox_command",
        executable="command_node",
        name="fox_command_node",
        output="screen",
    )
# Контроллер A1 через LCM
    a1_lcm_controller = Node(
        package="f_a1_lcm_control",
        executable="a1_lcm_controller",
        name="a1_lcm_controller",
        output="screen",
        parameters=[
            {"command_topic": "/f_fox_command/command"},
            # при желании можно сюда же положить другие параметры:
            # {"timer_dt": 0.01},
            # {"action_timeout": 1.5},
        ],
        # если хочешь, можешь сразу прописать LCM URL (иначе через env на хосте):
        # env={"LCM_DEFAULT_URL": "udpm://239.255.76.67:7667?ttl=0"},
    )


    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_cfg],
        output="screen",
    )

    return LaunchDescription([
        camera_name_arg,
        image_source_arg,
        camera_node,
        tf_launch,
        pose_node,
        gesture_classifier,
        fox_command_node,
        a1_lcm_controller,
        rviz_node,
    ])
