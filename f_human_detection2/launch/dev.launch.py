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

    camera_name = DeclareLaunchArgument(
        "camera_name",
        default_value="maf_camera",
        description="Maf-maf-maf",
    )

    image_source = DeclareLaunchArgument(
        "image_source",
        default_value="/dev/video0",
        description="Maf-maf",
    )


    camera_node=Node(
	    package='v4l2_camera',
	    executable='v4l2_camera_node',
	    name='v4l2_camera',
	    parameters=[
             {"camera_name": LaunchConfiguration("camera_name")},
            {"video_device": LaunchConfiguration("image_source")},
            {"io_method": "read"},          # <-- avoids mmap
            {"output_encoding": "rgb8"},    # matches your pipeline
            # {'camera_info_url': f'file://{calibration}'},
            # config_path
            {"frame_id" : "camera"}
            ],
	    output='screen'
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
        camera_name,
        camera_node,
        tf,
        pose,
        rviz,
        
    ])
