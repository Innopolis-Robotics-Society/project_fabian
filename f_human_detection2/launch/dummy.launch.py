from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    rviz_cfg = PathJoinSubstitution([FindPackageShare('f_human_detection2'), 'rviz', 'pose_debug.rviz'])
    params_yaml = PathJoinSubstitution([FindPackageShare('f_human_detection2'), 'config', 'dev.yaml'])


    # config_path = os.path.join(
    #     get_package_share_directory('rlr_camera'),
    #     'config',
    #     'param.yaml'
    # )

    image_source = DeclareLaunchArgument(
        "image_source",
        default_value="/dev/null",
        description="Maf-maf",
    )

    fps = DeclareLaunchArgument(
        "fps",
        default_value="30",
        description="Maf-maf",
    )

    video_node = Node(
        package="f_human_detection2",
        executable="video_publisher",
        name="video_publisher",
        parameters=[
            {"video_path": LaunchConfiguration("image_source")},
            {"fps": LaunchConfiguration("fps")},
        ],
        output="screen",
        remappings=[
            ("/image", "/image_raw")
        ],
    )
    

    pose = Node(
        package="f_human_detection2",
        executable="pose_node",
        name="f_human_detection2",
        parameters=[params_yaml],
        output="screen",
    )

    rviz = Node(
    package="rviz2",
    executable="rviz2",
    arguments=["-d", rviz_cfg],
    output="screen",
    )

    
    tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('f_bringup'), 'launch', 'tf.launch.py'])
        )
    )

    return LaunchDescription([
        image_source,
        fps,
        video_node,
        tf,
        pose,
        rviz,
        
    ])
